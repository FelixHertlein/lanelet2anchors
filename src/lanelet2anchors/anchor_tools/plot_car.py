import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib import transforms
from PIL import Image


def plot_rotated_image(image_path, x, y, angle, width, height):
    # prepare iamge
    img = Image.open(image_path)

    aspect_ratio = width / height
    if height > width:
        shape = int(img.height * aspect_ratio), img.height
    else:
        shape = img.width, int(img.width / aspect_ratio)

    img_quality = img.copy()
    img_quality = img_quality.resize(
        shape
    )  # sets aspect ratio while preserving the quality
    img_quality = img_quality.rotate(angle, expand=True)

    # setup figure
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # caclculate frame
    img = img.resize((width, height))
    img = img.rotate(angle, expand=True)

    offset_x = (img.width - width) / 2
    offset_y = (img.height - height) / 2
    extent = (
        x - offset_x,
        x + img.width - offset_x,
        y - offset_y,
        y + img.height - offset_y,
    )

    # reference rectangle
    rect = patches.Rectangle(
        (x, y), width, height, linewidth=1, edgecolor="r", facecolor="none"
    )
    rect_origin = (
        rect.get_x() + rect.get_width() / 2,
        rect.get_y() + rect.get_height() / 2,
    )
    t = (
        transforms.Affine2D().rotate_deg_around(*rect_origin, degrees=angle)
        + ax.transData
    )
    rect.set_transform(t)
    ax.add_patch(rect)

    # show image
    ax.imshow(img_quality, extent=extent)

    # axis settings
    ax.set_aspect("equal")
    ax.set_xlim(x - 100, x + img.width + 100)
    ax.set_ylim(y - 100, y + img.height + 100)
    plt.show()


plot_rotated_image(
    image_path="./car-top-view-icon.png", x=100, y=200, angle=30, width=50, height=100
)
