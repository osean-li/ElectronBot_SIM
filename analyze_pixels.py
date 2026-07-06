import numpy as np
from PIL import Image

img = np.array(Image.open('/tmp/electronbot_test_v3.png'))
print('shape', img.shape)
print('Bottom row sample every 20 pixels:')
for x in range(0, 640, 20):
    print(x, img[470, x].tolist())
