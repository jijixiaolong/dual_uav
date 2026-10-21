"""Condition-disjoint, train-only event-response prediction diagnostics.

These are diagnostics of recorded MuJoCo responses, not a learned controller.
Neither test responses nor oracle weld loads select the deployable model/gate.
"""
from dataclasses import dataclass
import hashlib
import json
from collections import defaultdict

import numpy as np


EVENT_NAMES = ('CONTACT', 'DOCK', 'UNDOCK')
DEFAULT_LAMBDAS = (1e-5, 1e-4, .001, .01, .1, 1., 10., 100.)


def mse_per_trial(actual, predicted):
    return np.mean((np.asarray(actual)-np.asarray(predicted))**2, axis=1)


@dataclass
class RidgeModel:
    mean: np.ndarray
    std: np.ndarray
    y_mean: np.ndarray
    A: np.ndarray
    regularization: float

    def predict(self, features):
        return ((np.asarray(features)-self.mean)/self.std) @ self.A + self.y_mean

    def to_dict(self):
        return dict(mean=self.mean.tolist(), std=self.std.tolist(), y_mean=self.y_mean.tolist(),
                    A=self.A.tolist(), lambda_=self.regularization,
                    prediction='((x - mean) / std) @ A + y_mean',
                    objective='mean_train_squared_residual_sum + lambda * squared_Frobenius_norm(A)',
                    normalization_fit='training split only')


def fit_ridge(features, responses, regularization):
    x, y = np.asarray(features, dtype=float), np.asarray(responses, dtype=float)
    if y.ndim == 1:
        y = y[:, None]
    if x.ndim != 2 or y.ndim != 2 or len(x) != len(y) or not len(x):
        raise ValueError('Expected nonempty 2D features and compatible responses')
    if regularization < 0 or not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('Data and regularization must be finite; lambda >= 0')
    mean, std, y_mean = x.mean(axis=0), x.std(axis=0), y.mean(axis=0)
    std = np.where(std > 1e-12, std, 1.)
    z = (x-mean)/std
    lhs, rhs = z.T@z + len(x)*regularization*np.eye(x.shape[1]), z.T@(y-y_mean)
    A = np.linalg.lstsq(lhs, rhs, rcond=None)[0] if regularization == 0 else np.linalg.solve(lhs, rhs)
    return RidgeModel(mean, std, y_mean, A, float(regularization))


def select_ridge(train_x, train_y, validation_x, validation_y, lambdas=DEFAULT_LAMBDAS):
    """No test data are accepted by this API. Ties prefer stronger shrinkage."""
    if not len(validation_x):
        raise ValueError('A separate validation split is required')
    candidates = []
    for regularization in sorted(set(lambdas)):
        model = fit_ridge(train_x, train_y, regularization)
        loss = float(mse_per_trial(validation_y, model.predict(validation_x)).mean())
        candidates.append((loss, -regularization, model))
    best = min(candidates, key=lambda item: (item[0], item[1]))
    return best[2], [dict(lambda_=item[2].regularization, validation_mse=item[0]) for item in candidates]


def validate_trial_splits(trials):
    by_condition = {}
    identities = set()
    for trial in trials:
        condition, split = str(trial['condition_id']), trial['split']
        if split not in ('train', 'validation', 'test'):
            raise ValueError(f'Unknown split {split!r}')
        if condition in by_condition and by_condition[condition] != split:
            raise ValueError(f'Condition leakage across splits: {condition}')
        by_condition[condition] = split
        identity = (condition, split, str(trial.get('seed')))
        if identity in identities:
            raise ValueError(f'Duplicate trial identity: {identity}')
        identities.add(identity)
    frozen_ids = sorted(identities)
    return hashlib.sha256(json.dumps(frozen_ids, separators=(',', ':')).encode()).hexdigest()


def _vector(value, size=None):
    try:
        result = np.asarray(value, dtype=float)
        if result.ndim == 1 and (size is None or len(result) == size) and np.isfinite(result).all():
            return result
    except (TypeError, ValueError):
        pass
    return None


def flatten_oracle(value):
    """Sorted numeric leaves only; booleans and metadata strings are not loads."""
    if isinstance(value, dict):
        return [x for key in sorted(value) for x in flatten_oracle(value[key])]
    if isinstance(value, (list, tuple)):
        return [x for item in value for x in flatten_oracle(item)]
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return [float(value)]
    return []


def extract_peak(metrics):
    if not isinstance(metrics, dict):
        return None
    for key in ('peak_attitude_error_deg', 'peak_so3_deg'):
        if key in metrics and isinstance(metrics[key], (float, int)) and np.isfinite(metrics[key]):
            return float(metrics[key])
    if isinstance(metrics.get('combined'), dict):
        combined = extract_peak(metrics['combined'])
        if combined is not None:
            return combined
    peaks = [extract_peak(value) for value in metrics.values() if isinstance(value, dict)]
    finite = [value for value in peaks if value is not None]
    return max(finite) if finite else None


def collect_event(trials, event_name):
    """Do not filter status. Missing events/windows remain in the audit table."""
    records, audit = [], []
    for trial in trials:
        event = trial.get('events', {}).get(event_name)
        identity = dict(condition_id=str(trial['condition_id']), split=trial['split'], seed=trial.get('seed'),
                        status=trial.get('status'), event=event_name)
        if not isinstance(event, dict):
            audit.append({**identity, 'eligible': False, 'reason': 'event_missing'})
            continue
        x, y = _vector(event.get('values32'), 32), _vector(event.get('response_vector'), 150)
        reason = ('features_missing_invalid_or_not_32' if x is None else
                  'response_missing_incomplete_invalid_or_not_150' if y is None else 'complete_event_window')
        audit.append({**identity, 'eligible': x is not None and y is not None, 'reason': reason})
        if x is not None and y is not None:
            oracle = _vector(flatten_oracle(event.get('oracle_loads')))
            records.append({**identity, 'x': x, 'y': y, 'peak': extract_peak(event.get('metrics', {})),
                            'oracle': oracle if oracle is not None and len(oracle) else None})
    return records, audit


def condition_bootstrap(reference_losses, model_losses, condition_ids, seed=0, replicates=2000):
    """Cluster bootstrap: average seeds within condition, then resample conditions."""
    grouped = defaultdict(list)
    for reference, model, condition in zip(reference_losses, model_losses, condition_ids):
        grouped[str(condition)].append((reference, model))
    if not grouped:
        return dict(conditions=0, improvement_pct=None, improvement_ci95_pct=None)
    losses = np.array([np.mean(grouped[key], axis=0) for key in sorted(grouped)])
    rng = np.random.default_rng(seed)
    samples = losses[rng.integers(0, len(losses), size=(replicates, len(losses)))].mean(axis=1)
    delta = samples[:, 0]-samples[:, 1]
    safe = samples[:, 0] > 1e-20
    ratios = 100*delta[safe]/samples[safe, 0]
    point = losses.mean(axis=0)
    return dict(conditions=len(losses), replicates=replicates,
                reference_condition_mean_mse=float(point[0]), model_condition_mean_mse=float(point[1]),
                improvement_pct=float(100*(point[0]-point[1])/point[0]) if point[0] > 1e-20 else None,
                improvement_ci95_pct=np.percentile(ratios, [2.5, 97.5]).tolist() if len(ratios) else None,
                mse_difference_ci95=np.percentile(delta, [2.5, 97.5]).tolist(),
                sampling_unit='condition; seeds within each condition averaged first')


def _correlation(a, b):
    a, b = np.asarray(a).ravel(), np.asarray(b).ravel()
    if len(a) < 3 or a.std() < 1e-15 or b.std() < 1e-15:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def nearest_neighbor_diagnostic(model, train_x, train_y, heldout_x, heldout_y):
    train_z, query_z = (train_x-model.mean)/model.std, (heldout_x-model.mean)/model.std
    feature_distance = np.linalg.norm(query_z[:, None, :]-train_z[None, :, :], axis=2)/np.sqrt(train_z.shape[1])
    response_mse = np.mean((heldout_y[:, None, :]-train_y[None, :, :])**2, axis=2)
    nearest = np.argmin(feature_distance, axis=1)
    near_loss = response_mse[np.arange(len(heldout_x)), nearest]
    random_loss = response_mse.mean(axis=1)  # exact expected random training neighbor
    from scipy.stats import rankdata
    result = dict(feature_response_distance_pearson=_correlation(feature_distance, np.sqrt(response_mse)),
                  feature_response_distance_spearman=_correlation(rankdata(feature_distance.ravel()), rankdata(response_mse.ravel())),
                  nearest_response_mse=float(near_loss.mean()), random_neighbor_response_mse=float(random_loss.mean()),
                  nearest_to_random_mse_ratio=float(near_loss.mean()/random_loss.mean()) if random_loss.mean() > 1e-20 else None,
                  nearest_feature_distance=float(feature_distance[np.arange(len(heldout_x)), nearest].mean()),
                  pair_count=int(feature_distance.size),
                  correlation_scope='descriptive only; pair distances are dependent, no iid p-value',
                  random_control='exact mean over every training neighbor, not selected using response')
    per_query = [dict(nearest_train_index=int(nearest[i]), nearest_feature_distance=float(feature_distance[i, nearest[i]]),
                      nearest_response_mse=float(near_loss[i]), random_neighbor_response_mse=float(random_loss[i]))
                 for i in range(len(heldout_x))]
    return result, per_query


def _r2(y, predicted):
    denominator = float(np.sum((y-y.mean(axis=0))**2))
    return float(1.-np.sum((y-predicted)**2)/denominator) if denominator > 1e-20 else None


def analyze_event(records, feature_variant='raw32', lambdas=DEFAULT_LAMBDAS, gate_fraction=.05,
                  seed=20261021, bootstrap_replicates=2000, label_shuffles=64):
    """Freeze all choices on validation, then evaluate validation/test once each."""
    grouped = {split: [r for r in records if r['split'] == split] for split in ('train', 'validation', 'test')}
    counts = {split: len(value) for split, value in grouped.items()}
    minimum = dict(train=3, validation=2, test=1)
    if any(counts[split] < count for split, count in minimum.items()):
        return dict(status='insufficient_complete_windows', counts=counts,
                    minimum_counts=minimum, validation_gate=False), {}, []
    if feature_variant == 'oracle':
        widths = {len(r['oracle']) if r['oracle'] is not None else 0 for r in records}
        if len(widths) != 1 or 0 in widths:
            return dict(status='oracle_loads_missing_or_inconsistent', counts=counts, validation_gate=False), {}, []
    features = lambda r: np.r_[r['x'], r['oracle']] if feature_variant == 'oracle' else r['x']
    arrays = {split: (np.array([features(r) for r in subset]), np.array([r['y'] for r in subset]))
              for split, subset in grouped.items()}
    train_x, train_y = arrays['train']; validation_x, validation_y = arrays['validation']
    model, candidates = select_ridge(train_x, train_y, validation_x, validation_y, lambdas)
    train_variance = float(np.mean((train_y-model.y_mean)**2))
    output = dict(status='evaluated', counts=counts, feature_variant=feature_variant,
                  feature_dimension=train_x.shape[1], selected_lambda=model.regularization,
                  lambda_candidates=candidates, train_response_variance=train_variance,
                  nmse_denominator='training response variance, mean over all 150 response coordinates',
                  gate_threshold_fraction=gate_fraction,
                  selection_scope='training fit and normalization; validation hyperparameter/gate; test once after freeze',
                  response_units='mean square of supplied mixed-unit response: roll/pitch rad and 0.5*position m')
    payload, prediction_rows = {'response': model}, []
    rng = np.random.default_rng(seed)
    shuffled_models = [fit_ridge(train_x, train_y[rng.permutation(len(train_y))], model.regularization)
                       for _ in range(label_shuffles)]
    for split in ('validation', 'test'):
        x, y = arrays[split]
        predicted = model.predict(x); mean_prediction = np.broadcast_to(model.y_mean, y.shape)
        losses, reference_losses = mse_per_trial(y, predicted), mse_per_trial(y, mean_prediction)
        bootstrap = condition_bootstrap(reference_losses, losses, [r['condition_id'] for r in grouped[split]],
                                        seed=seed, replicates=bootstrap_replicates)
        near, near_rows = nearest_neighbor_diagnostic(model, train_x, train_y, x, y)
        shuffle_predictions = np.array([m.predict(x) for m in shuffled_models])
        shuffle_losses = np.mean((shuffle_predictions-y[None, :, :])**2, axis=2)
        result = dict(mse=float(losses.mean()), mean_predictor_mse=float(reference_losses.mean()),
                      nmse=float(losses.mean()/train_variance) if train_variance > 1e-20 else None,
                      mean_predictor_nmse=float(reference_losses.mean()/train_variance) if train_variance > 1e-20 else None,
                      relative_improvement_fraction=float(1.-losses.mean()/reference_losses.mean()) if reference_losses.mean() > 1e-20 else None,
                      condition_bootstrap=bootstrap, nearest_neighbor=near,
                      training_label_shuffle=dict(repeats=label_shuffles, same_selected_lambda=True,
                           mse_mean=float(shuffle_losses.mean()),
                           mse_ci95_across_shuffles=np.percentile(shuffle_losses.mean(axis=1), [2.5, 97.5]).tolist(),
                           observed_model_better_fraction=float(np.mean(shuffle_losses.mean(axis=1) > losses.mean()))))
        output[split] = result
        for i, record in enumerate(grouped[split]):
            prediction_rows.append({key: record[key] for key in ('condition_id', 'split', 'seed', 'status')})
            prediction_rows[-1].update(response_mse=float(losses[i]), mean_predictor_mse=float(reference_losses[i]),
                                       label_shuffle_mean_mse=float(shuffle_losses[:, i].mean()),
                                       **near_rows[i], predicted_response=predicted[i].tolist(), actual_response=y[i].tolist())
    validation_improvement = output['validation']['relative_improvement_fraction']
    output['validation_gate'] = bool(feature_variant == 'raw32' and validation_improvement is not None
                                     and validation_improvement >= gate_fraction)
    output['oracle_gate_ignored'] = feature_variant == 'oracle'
    peak_subsets = {split: [r for r in subset if r['peak'] is not None] for split, subset in grouped.items()}
    if all(len(peak_subsets[s]) >= minimum[s] for s in minimum):
        peak_x = {s: np.array([features(r) for r in subset]) for s, subset in peak_subsets.items()}
        peak_y = {s: np.array([[r['peak']] for r in subset]) for s, subset in peak_subsets.items()}
        peak_model, peak_candidates = select_ridge(peak_x['train'], peak_y['train'], peak_x['validation'], peak_y['validation'], lambdas)
        payload['peak'] = peak_model
        output['peak_so3_prediction'] = dict(units='deg', selected_lambda=peak_model.regularization,
                                            lambda_candidates=peak_candidates, counts={s:len(v) for s,v in peak_subsets.items()})
        for split in ('validation', 'test'):
            actual, predicted = peak_y[split], peak_model.predict(peak_x[split])
            baseline = np.broadcast_to(peak_model.y_mean, actual.shape)
            output['peak_so3_prediction'][split] = dict(r2=_r2(actual, predicted),
                    mean_predictor_r2=_r2(actual, baseline), mse_deg2=float(np.mean((actual-predicted)**2)))
    else:
        output['peak_so3_prediction'] = dict(status='insufficient_scalar_labels',
                                            counts={s: len(v) for s,v in peak_subsets.items()})
    return output, payload, prediction_rows
