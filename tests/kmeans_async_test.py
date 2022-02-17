import unittest
import cv2
import time


from .context import r2_object_detection


find_ks = r2_object_detection.kmeans_async.find_ks
cv_kmeans = r2_object_detection.kmeans.cv_kmeans

# from r2_object_detection.kmeans_async import find_ks
# from r2_object_detection.kmeans import cv_kmeans


class KmeansAsyncTests(unittest.TestCase):

    def setUp(self) -> None:
        self.ks = [3, 5, 10]
        self.org_img = cv2.imread('test_ord_img.jpg')
        self.rgbd = cv2.imread('test_rgbd.jpg')

    def test_consistency(self):
        """
        This test is only meant to test that the results are consistent with
        non-asynchronous (synchronous) calls.
        """
        ks = self.ks
        org_img = self.org_img
        rgbd = self.rgbd
        async_kmeans = find_ks(org_img, rgbd, ks, viz=False)
        sync_kmeans = {k: cv_kmeans(org_img, k) for k in ks}
        self.assertEqual(async_kmeans, sync_kmeans, 'Asynchronous kmeans are '
                                                    'not consistent with '
                                                    'synchronous kmeans.')

    def test_speed(self):
        """
        This test is not the most precise- this is okay. In production, we only
        want to use async if it is faster. This test gives us a prompt
        response.
        """
        ks = self.ks
        org_img = self.org_img
        rgbd = self.rgbd
        start = time.time()
        async_kmeans = set(find_ks(org_img, rgbd, ks, viz=False))
        end = time.time()
        async_time = end-start
        start = time.time()
        sync_kmeans = {cv_kmeans(org_img, k) for k in ks}
        end = time.time()
        sync_time = end-start
        self.assertGreater(sync_time, async_kmeans, 'Asynchronous kmeans took '
                                                    'longer to generate than'
                                                    'synchronous kmeans.')


if __name__ == '__main__':
    unittest.main()
