from multiprocessing import Pool
from typing import TYPE_CHECKING, Iterable

from kmeans import cv_kmeans, viz_image


if TYPE_CHECKING:
    import numpy


DEFAULT_Ks = [5]


def find_ks(org_image: numpy.ndarray, rgbd: numpy.ndarray, pool: Pool = None,
            ks: Iterable[int] = None, workers: int = 4, viz : bool = False):
    """
    Visualizes optimal k by running `kmeans.py` on all values of `k` in
    iterable `ks`.

    PARAMETERS
    ----------
    org_image
        org_image
    rgbd
        A numpy.ndarray of red, green, blue, and depth frames.
    pool
        A multiprocessing.Pool of workers.
    ks
        A set of iterable Ks to run analysis on.
    workers
        Number of worker processes to use when analysing the Ks.
    viz
        Whether to visualize the images generated
    """
    if pool is None:
        pool = Pool(processes=workers)
    if ks is None:
        ks = DEFAULT_Ks.copy()
    kmean_futures = [pool.apply_async(cv_kmeans, (rgbd, k))
                     for k in ks]
    kmeans = {ks[i]: kmeans.get() for i, kmeans in enumerate(kmean_futures)}
    if viz:
        for i, kmean in enumerate(kmeans):
            viz_image([org_image, kmean, rgbd],
                      ['Original', f'Result (k={ks[i]})', 'Depth Frame'])
    return kmeans




