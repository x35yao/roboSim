import pickle
import pprint as pp
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from sklearn.cluster import AgglomerativeClustering
from sklearn.covariance import MinCovDet #TODO: reconsider this in case we have multimodal distributions
from scipy.stats import trim_mean


seqs = []


minimum_demonstration_path = '../sim_data/1599153598/data.pickle' # Data with only one bolt and one nut

with open(minimum_demonstration_path, "rb") as logfile:
    while True:
        try:
            seqs.append(pickle.load(logfile))
        except EOFError:
            break

logfile_path = '../sim_data/1599166289/data.pickle'
with open(logfile_path, "rb") as logfile:
    while True:
        try:
            seqs.append(pickle.load(logfile))
        except EOFError:
            break

seqs_success = []
for seq in seqs:
    if len(seq) == 4:
        seqs_success.append(seq)

# Clustering

def p_norm_distance(x, y, p):
    difference = np.abs(x - y)
    return np.sum(difference ** p) ** (1 / p)

def clustering(data):
    # compute distances using p-norm with -ve exponent
    # note exponent can't be too large; we want closeness in multiple dimensions to be somewhat better than closeness in only one
    n = data.shape[1]
    distances = np.zeros((n,n))
    for i in range(n):
        for j in range(i+1,n):
            distances[i,j] = p_norm_distance(data[:,i], data[:,j], p)
    distances = distances + distances.T

    # cluster using these distances
    ac = AgglomerativeClustering(n_clusters=None, affinity='precomputed', linkage='average', distance_threshold= thres)
    estimated_labels = ac.fit_predict(distances)
    return estimated_labels

def build_model(estimated_labels, data_concat):
    models = []
    # model the distribution of each cluster robustly in case there are misclassifications
    for label in np.unique(estimated_labels):
        indices = np.nonzero(estimated_labels == label)
        samples = data_concat[:, indices].squeeze()
    #     print('Group {} means and standard deviations'.format(label))
        means = trim_mean(samples, .1, axis=1)
        mcd = MinCovDet(support_fraction=.95).fit(samples.T)
        standard_deviations = np.diag(mcd.covariance_)**.5
        models.append(means)
    return models

def pred_label(pos, models):
    dists = []
    for model in models:
        means = model[0]
        # covirance = model[1]
        distance = p_norm_distance(pos, means, p = -2)
        dists.append(distance)
    model_ind = np.argmin(dists)
    return model_ind

def get_global_state_vec(seqs, models_nut, models_bolt):
    num_global_states = len(seqs[0])
    num_local_states = len(models_nut + models_bolt)
    state_arr = np.zeros((num_global_states, num_local_states))
    for seq in seqs:
        if len(seq) == 4: # No failure trial
            for i,step in enumerate(seq):
                arr = np.zeros(num_local_states)
                for obj in step['objs']:
                    pos = obj['pos']
                    if obj['class'] == 'nut':
                        ind = pred_label(pos, models_nut)
                        arr[ind] = 1
                    elif obj['class'] == 'bolt':
                        ind = pred_label(pos, models_bolt)
                        arr[ind + len(models_nut)] = 1
                state_arr[i] = state_arr[i] + arr

    return (state_arr == np.max(state_arr)).astype(int)

p = -3
thres = 0.001
num_data = 200
data_nut = []
data_bolt = []
for seq in seqs_success:
    if len(seq) == 4: # Sequence without failure
        for step in seq:
            pos_nut = [obj['pos'] for obj in step['objs'] if obj['class'] == 'nut']
            pos_bolt = [obj['pos'] for obj in step['objs'] if obj['class'] == 'bolt']
            data_nut += pos_nut
            data_bolt += pos_bolt
data_nut = np.asarray(data_nut).T
data_bolt = np.asarray(data_bolt).T

estimated_labels_nut = clustering(data_nut)
estimated_labels_bolt = clustering(data_bolt)

models_nut = build_model(estimated_labels_nut, data_nut)
models_bolt = build_model(estimated_labels_bolt, data_bolt)
global_state_vec = get_global_state_vec(seqs, models_nut, models_bolt)
np.save('models_nut.npy', np.asarray(models_nut))
np.save('models_bolt.npy', np.asarray(models_bolt))
np.save('global_state_vec.npy', np.asarray(global_state_vec))
