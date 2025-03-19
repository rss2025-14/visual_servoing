import cv2
import numpy as np
import os

# Ensure the folder exists
# SAVE_DIR = "color_segmentation_tests_images"
# if not os.path.exists(SAVE_DIR):
   # os.makedirs(SAVE_DIR)

# Global counter for naming images
# image_counter = 0

def image_print(img):
    """
    Helper function to display images for debugging. Press any key to continue.
    """
    # global image_counter
    # image_counter += 1
    # filename = os.path.join(SAVE_DIR, f"test_cone_{image_counter}.png")
    # cv2.imwrite(filename, img)  # Save the image

    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def get_average_color(img, bbox):
    """
    Calculate the average color inside the bounding box.

    Input:
        img: np.3darray; input image in BGR format.
        bbox: ((x1, y1), (x2, y2)) - Bounding box coordinates.

    Return:
        avg_color: (B, G, R) tuple representing the average color.
    """
    (x1, y1), (x2, y2) = bbox
    roi = img[y1:y2, x1:x2]  # Extract the region of interest

    if roi.size == 0:  # Check if ROI is empty
        return (0, 0, 0)

    avg_color = np.mean(roi, axis=(0, 1))  # Compute the average color
    return tuple(map(int, avg_color))  # Convert to integer values

def cd_color_segmentation(img, template=None):
    """
    Detect the cone using color segmentation and display the result with a bounding box and color text.

    Input:
        img: np.3darray; input image in BGR format.
        template: Optional, can be used to automate setting hue filter values.

    Return:
        bbox: ((x1, y1), (x2, y2)) - Bounding box of the detected cone.
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range for orange color in HSV
    lower_orange = np.array([5, 170, 170])
    upper_orange = np.array([20, 255, 255])

    # lower_orange = np.array([0, 220, 100])  
    # upper_orange = np.array([30, 255, 255])

    # Create a mask
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_bbox = ((0, 0), (0, 0))  # Default bounding box if no valid object is found
    best_avg_color = (0, 0, 0)

    if contours:
        # Sort contours by area (largest first)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for contour in contours:
            # Get bounding box for the contour
            x, y, w, h = cv2.boundingRect(contour)
            bounding_box = ((x, y), (x + w, y + h))

            # Compute the average color inside the bounding box
            avg_color = get_average_color(img, bounding_box)

            # Check if the object's brightness is lower than 60
            if avg_color[0] < 60:  # R channel in BGR
                best_bbox = bounding_box
                best_avg_color = avg_color
                break  # Stop at the first (largest) valid object

        if best_bbox != ((0, 0), (0, 0)):  # If a valid cone is found
            (x1, y1), (x2, y2) = best_bbox

            # Draw the bounding box on the image
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Display the average color as text
            text = f"Color: {best_avg_color[::-1]}"  # Convert BGR to RGB for readability
            cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Show the image with the bounding box and color text
    # image_print(img)

    return best_bbox
