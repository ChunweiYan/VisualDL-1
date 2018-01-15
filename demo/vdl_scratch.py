#!/user/bin/env python
import os
from visualdl import LogWriter, ROOT
import subprocess
import math
from scipy.stats import norm
import numpy as np
import random
from PIL import Image

logdir = './scratch_log'

logw = LogWriter(logdir, sync_cycle=30)

# create scalars in mode train and test.
with logw.mode('train') as logger:
    scalar0 = logger.scalar("scratch/scalar")

with logw.mode('test') as logger:
    scalar1 = logger.scalar("scratch/scalar")

# add scalar records.
last_record0 = 0.
last_record1 = 0.
for step in range(1, 100):
    last_record0 += 0.1 * (random.random() - 0.3)
    last_record1 += 0.1 * (random.random() - 0.7)
    scalar0.add_record(step, last_record0)
    scalar1.add_record(step, last_record1)

# create histogram
with logw.mode('train') as logger:
    histogram = logger.histogram("scratch/histogram", num_buckets=200)
    for step in range(1, 100):
        histogram.add_record(step,
                             np.random.normal(
                                 0.1 + step * 0.001,
                                 200. / (100 + step),
                                 size=1000))

# create image
with logw.mode("train") as logger:
    image = logger.image("scratch/dog", 4,
                         1)  # randomly sample 4 images one pass
    dog_jpg = Image.open(os.path.join(ROOT, 'python/dog.jpg'))
    dog_jpg = dog_jpg.resize(np.array(dog_jpg.size) / 2)
    shape = [dog_jpg.size[1], dog_jpg.size[0], 3]

    for pass_ in xrange(4):
        image.start_sampling()
        for sample in xrange(10):
            # randomly crop a demo image.
            target_shape = [100, 100, 3]  # width, height, channels(3 for RGB)
            left_x = random.randint(0, shape[1] - target_shape[1])
            left_y = random.randint(0, shape[0] - target_shape[0])
            right_x = left_x + target_shape[1]
            right_y = left_y + target_shape[0]

            idx = image.is_sample_taken()
            # a more efficient way to sample images is
            if idx >= 0:
                data = np.array(
                    dog_jpg.crop((left_x, left_y, right_x,
                                  right_y))).flatten()
                image.set_sample(idx, target_shape, data)
            # you can also just write followig codes, it is more clear, but need to
            # process image even if it will not be sampled.
            # data = np.array(
            #     dog_jpg.crop((left_x, left_y, right_x,
            #                     right_y))).flatten()
            # image.add_sample(shape, data)

        image.finish_sampling()
