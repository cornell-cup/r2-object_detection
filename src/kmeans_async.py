from multiprocessing import Pool

from kmeans import cv_kmeans, viz_image


DEFAULT_Ks = [5]


def find_ks(org_image, rgbd, pool=None, ks=None, workers=4):
    """
    Visualizes optimal k by running `kmeans.py` on all values of `k` in
    iterable `ks`.

    PARAMETERS
    ----------
    org_image -
        org_image
    rgbd -
        A numpy array of red, green, blue, and depth frames.
    pool -
        A multiprocessing.Pool pool of workers.
    ks -
        A set of iterable Ks to run analysis on.
    workers -
        Number of worker processes to use when analysing the Ks.
    """
    if pool is None:
        pool = Pool(processes=workers)
    if ks is None:
        ks = DEFAULT_Ks.copy()
    kmean_futures = [pool.apply_async(cv_kmeans, (rgbd, k))
                     for k in ks]
    for i, kmeans in enumerate(kmean_futures):
        viz_image([org_image, kmeans.get(), rgbd],
                  ['Original', f'Result (k={ks[i]})', 'Depth Frame'])


