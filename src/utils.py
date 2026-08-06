import numpy as np
import matplotlib
matplotlib.use('Agg')  # Don't show img in the call
import matplotlib.pyplot as plt

np.random.seed(3)

def show_mask(mask, ax, random_color=False, borders = True):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask = mask.astype(np.uint8)
    mask_image =  mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    if borders:
        import cv2
        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        # Try to smooth contours
        contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
        mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2) 
    ax.imshow(mask_image)

def show_points(coords, ax, marker_size=375):
    ax.scatter(coords[:, 0], coords[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)

def show_box(box, ax, label=None):
    box = np.asarray(box).reshape(-1)
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    ax.add_patch(
        plt.Rectangle(
            (x0, y0),
            w,
            h,
            edgecolor='green',
            facecolor=(0, 0, 0, 0),
            lw=2,
            clip_on=True,
        )
    )
    if label is not None:
        image_left, image_right = sorted(ax.get_xlim())
        image_top, image_bottom = sorted(ax.get_ylim())
        if (x0 + x1) / 2 > (image_left + image_right) / 2:
            label_x = min(x1, image_right)
            horizontal_alignment = 'right'
        else:
            label_x = max(x0, image_left)
            horizontal_alignment = 'left'
        if y0 - image_top < 16:
            label_y = max(y0, image_top)
            vertical_alignment = 'top'
        else:
            label_y = y0
            vertical_alignment = 'bottom'
        ax.text(
            label_x,
            label_y,
            str(label),
            color='white',
            fontsize=10,
            horizontalalignment=horizontal_alignment,
            verticalalignment=vertical_alignment,
            bbox=dict(facecolor='green', alpha=0.8, pad=2, edgecolor='none'),
            clip_on=True,
        )

def show_fig(
    image,
    save_path,
    masks=None,
    point_coords=None,
    box_coords=None,
    box_labels=None,
    borders=True,
):
    image = np.asarray(image)
    image_height, image_width = image.shape[:2]
    dpi = 100
    figure = plt.figure(
        figsize=(image_width / dpi, image_height / dpi),
        dpi=dpi,
        frameon=False,
    )
    axis = figure.add_axes([0, 0, 1, 1])
    axis.imshow(image)
    axis.set_xlim(-0.5, image_width - 0.5)
    axis.set_ylim(image_height - 0.5, -0.5)

    if masks is not None:
        for mask in masks:
            show_mask(mask, axis, borders=borders)

    if point_coords is not None:
        show_points(point_coords, axis)

    if box_coords is not None:
        boxes = np.asarray(box_coords)
        if boxes.shape == (2, 2):
            boxes = boxes.reshape(1, 4)
        elif boxes.ndim == 1:
            boxes = boxes.reshape(1, 4)
        labels = (
            [None] * len(boxes)
            if box_labels is None
            else list(box_labels)
        )
        if len(labels) != len(boxes):
            raise ValueError("box_labels must match the number of boxes")
        for box, label in zip(boxes, labels):
            show_box(box, axis, label=label)

    axis.axis('off')
    figure.savefig(
        save_path,
        pad_inches=0,
        dpi=dpi,
    )
    plt.close(figure)
    print('[VIS] Saved in', save_path)
