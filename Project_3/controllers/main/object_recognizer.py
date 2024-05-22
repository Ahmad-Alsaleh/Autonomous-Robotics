from typing import Literal
import numpy as np
import cv2
import random
import torch


# -------------------------------------
# Functions for object recognition
# -------------------------------------
def read_image(path):
    img = cv2.imread(path)
    if img is None:
        print("Image not found")
        return None
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img_gray


def SIFT(img):
    # siftDetector= cv2.SIFT_create(nfeatures=100, nOctaveLayers=4, contrastThreshold=0.04, edgeThreshold=1, sigma=1.6)
    siftDetector = cv2.SIFT_create()
    kp, des = siftDetector.detectAndCompute(img, None)
    return kp, des


def homography(pairs):
    rows = []
    for i in range(pairs.shape[0]):
        p1 = np.append(pairs[i][0:2], 1)
        p2 = np.append(pairs[i][2:4], 1)
        row1 = [
            0,
            0,
            0,
            p1[0],
            p1[1],
            p1[2],
            -p2[1] * p1[0],
            -p2[1] * p1[1],
            -p2[1] * p1[2],
        ]
        row2 = [
            p1[0],
            p1[1],
            p1[2],
            0,
            0,
            0,
            -p2[0] * p1[0],
            -p2[0] * p1[1],
            -p2[0] * p1[2],
        ]
        rows.append(row1)
        rows.append(row2)
    rows = np.array(rows)
    U, s, V = np.linalg.svd(rows)
    H = V[-1].reshape(3, 3)
    H = H / H[2, 2]  # standardize to let w*H[2,2] = 1
    return H


def random_point(matches, k=4):
    idx = random.sample(range(len(matches)), k)
    point = [matches[i] for i in idx]
    return np.array(point)


def get_error(points, H):
    num_points = len(points)
    all_p1 = np.concatenate((points[:, 0:2], np.ones((num_points, 1))), axis=1)
    all_p2 = points[:, 2:4]
    estimate_p2 = np.zeros((num_points, 2))
    for i in range(num_points):
        temp = np.dot(H, all_p1[i])
        estimate_p2[i] = (temp / temp[2])[0:2]
    errors = np.linalg.norm(all_p2 - estimate_p2, axis=1) ** 2
    return errors


def ransac(matches, threshold, iters):
    best_inliers = None
    best_H = None
    num_best_inliers = 0
    for i in range(iters):
        points = random_point(matches)
        H = homography(points)
        #  avoid dividing by zero
        if np.linalg.matrix_rank(H) < 3:
            continue
        errors = get_error(matches, H)
        idx = np.where(errors < threshold)[0]
        inliers = matches[idx]
        num_inliers = len(inliers)
        if num_inliers > num_best_inliers:
            best_inliers = inliers.copy()
            num_best_inliers = num_inliers
            best_H = H.copy()
    return best_inliers, best_H


def find_position(H, image_left, image_right):
    if H is None:
        return None, None, None, None
    h, w = image_left.shape
    corners = np.array([[0, 0, 1], [0, h, 1], [w, 0, 1], [w, h, 1]])
    corners = np.dot(H, corners.T).T
    corners = corners / corners[:, 2].reshape(-1, 1)
    x_min, y_min = np.min(corners[:, 0]), np.min(corners[:, 1])
    x_max, y_max = np.max(corners[:, 0]), np.max(corners[:, 1])
    return x_min, y_min, x_max, y_max


# -------------------------------------
# End of functions for object recognition
# -------------------------------------


# -------------------------------------
# ObjectRecognizer class
# -------------------------------------
class ObjectRecognizer:
    def __init__(self) -> None:
        self.__target_image_gray = read_image("target_object2_nobg.png")
        self.__kp_target, self.__des_target = SIFT(self.__target_image_gray)
        self.__k = 4  # hyperparameter
        self.__target_size = 0.0635  # in meters, must be known in advance
        self.__fov = 1.57  # in radian
        self.__focal_length = self.__target_image_gray.shape[1] / (
            2 * np.tan(self.__fov / 2)
        )
        self.yolo_model = torch.hub.load(
            "ultralytics/yolov5", "yolov5s", pretrained=True, force_reload=False
        ).to("cpu")

    def compute_distance_and_angle(self, x_min, x_max, scene_image):
        """Computes the distance and angle of the target object in the scene image from the coordinates"""
        if x_min is None:
            return None, None
        target_size = x_max - x_min
        distance = self.__focal_length * self.__target_size / target_size
        image_center = scene_image.shape[1] / 2
        target_center = (x_min + x_max) / 2
        angle = (target_center - image_center) / image_center * (self.__fov / 2)
        return distance, angle

    def detect_objects(
        self, scene_image: np.ndarray, *, model: Literal["YOLO", "SWIFT"]
    ) -> np.ndarray:
        if model == "YOLO":
            return self.__detect_objects_yolo(scene_image)
        elif model == "SWIFT":
            return self.__detect_objects_swift(scene_image)
        else:
            raise ValueError("Invalid object detection algorithm")

    def __detect_objects_swift(self, scene_image: np.ndarray) -> np.ndarray:
        """Detects target objects in the image and returns the coordinates of the object(s) if any."""
        if scene_image is None:
            raise ValueError("Image not found")
        scene_image = cv2.cvtColor(scene_image, cv2.COLOR_RGB2GRAY)
        kp_scene, des_scene = SIFT(scene_image)
        if des_scene is None:
            return None
        kp_target, des_target = self.__kp_target, np.float32(self.__des_target)
        des_scene = np.float32(des_scene)

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des_target, des_scene, k=2)  # find the best two matches
        if len(matches) < 2:
            return None
        good = []
        for match in matches:
            if len(match) < 2:
                continue
            m, n = match
            if m.distance < 0.49 * n.distance:
                good.append(m)
        good = np.array(good)
        if len(good) < self.__k:
            return None
        matches_param = np.array(
            [
                [
                    kp_target[m.queryIdx].pt[0],
                    kp_target[m.queryIdx].pt[1],
                    kp_scene[m.trainIdx].pt[0],
                    kp_scene[m.trainIdx].pt[1],
                ]
                for m in good
            ]
        )
        _, best_H = ransac(matches_param, 10, 1000)
        x_min, y_min, x_max, y_max = find_position(
            best_H, self.__target_image_gray, scene_image
        )
        if x_max and x_min and (x_max - x_min) < 0.01:
            return None
        distance, angle = self.compute_distance_and_angle(x_min, x_max, scene_image)
        if distance is None:
            return None
        cv2.imwrite("test.png", scene_image)  # save the image for debugging
        return [(distance, angle)]

    def __detect_objects_yolo(self, scene_image: np.ndarray) -> np.ndarray:
        """Detects target objects in the image and returns the coordinates of the object(s) if any."""
        scene_image = cv2.cvtColor(scene_image, cv2.COLOR_RGB2BGR)
        results = self.yolo_model(scene_image)
        results.render()
        if len(results.xyxy[0]) == 0:
            return None
        x_min = results.xyxy[0][0][0]
        x_max = results.xyxy[0][0][2]
        distance, angle = self.compute_distance_and_angle(x_min, x_max, scene_image)
        class_name = results.names[int(results.xyxy[0][0][5])]
        # print(class_name) # to show the class name of the detected object
        if (class_name != "cup" and class_name != "bottle") or distance is None:
            return None
        cv2.imwrite(r"test.jpg", scene_image)  # save the image with boxes
        return [(distance, angle)]
