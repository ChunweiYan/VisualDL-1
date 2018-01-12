#ifndef VISUALDL_UTILS_IMAGE_H
#define VISUALDL_UTILS_IMAGE_H

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "visualdl/utils/logging.h"

namespace visualdl {

using uint8_t = unsigned char;

/*
 * 2: height*width, channel
 */
template <typename T>
using ImageDT =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Uint8Image = ImageDT<uint8_t>;

/**
 * Bilinear resize grayscale image.
 * pixels is an array of size w * h.
 * Target dimension is w2 * h2.
 * w2 * h2 cannot be zero.
 *
 * @param pixels Image pixels.
 * @param w Image width.
 * @param h Image height.
 * @param w2 New width.
 * @param h2 New height.
 * @return New array with size w2 * h2.
 */
void ResizeBilinearGray(const uint8_t* pixels,
                        int w,
                        int h,
                        int w2,
                        int h2,
                        std::vector<uint8_t>& temp) {
  LOG(INFO) << "scaling image from " << w << " " << h << " to " << w2 << " "
            << h2;
  int A, B, C, D, x, y, index, gray;
  float x_ratio = ((float)(w - 1)) / w2;
  float y_ratio = ((float)(h - 1)) / h2;
  float x_diff, y_diff, ya, yb;
  int offset = 0;
  for (int i = 0; i < h2; i++) {
    for (int j = 0; j < w2; j++) {
      x = (int)(x_ratio * j);
      y = (int)(y_ratio * i);
      x_diff = (x_ratio * j) - x;
      y_diff = (y_ratio * i) - y;
      index = y * w + x;

      // range is 0 to 255 thus bitwise AND with 0xff
      A = pixels[index] & 0xff;
      B = pixels[index + 1] & 0xff;
      C = pixels[index + w] & 0xff;
      D = pixels[index + w + 1] & 0xff;

      // Y = A(1-w)(1-h) + B(w)(1-h) + C(h)(1-w) + Dwh
      gray =
          (int)(A * (1 - x_diff) * (1 - y_diff) + B * (x_diff) * (1 - y_diff) +
                C * (y_diff) * (1 - x_diff) + D * (x_diff * y_diff));

      temp[offset++] = gray;
    }
  }
}

/*
 * Rescale image's size.
 */
static void RescaleImage(const uint8_t* source,
                         std::vector<uint8_t>* target,
                         int width,
                         int height,
                         int depth,
                         int target_width,
                         int target_height) {
  CHECK_LE(target_width, 600) << "too large width to rescale image";
  CHECK_LE(target_height, 800) << "too large height to rescale image";
  target->resize(target_width * target_height * depth);

  float width_scale = (float)width / target_width;
  float height_scale = (float)height / target_height;

  LOG(INFO) << "width scale: " << width_scale;
  LOG(INFO) << "height scale: " << height_scale;
  LOG(INFO) << "target_width: " << target_width;
  LOG(INFO) << "target_height: " << target_height;
  LOG(INFO) << "sid size:" << width * height * depth;
  LOG(INFO) << "tid size:" << target_width * target_height * depth;
  LOG(INFO) << "depth: " << depth;

  std::vector<uint8_t> temp(target_width * target_height);
  for (int cn = 0; cn < depth; cn++) {
    const uint8_t* image = source + width * height * cn;
    ResizeBilinearGray(image, width, height, target_width, target_height, temp);
  }
}

/*
 * hw: height*width
 * depth: number of channels
 */
static void NormalizeImage(Uint8Image* image,
                           const float* buffer,
                           int hw,
                           int depth) {
  // Both image and buffer should be used in row major.
  Eigen::Map<const Eigen::
                 Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      values(buffer, depth, hw);

  CHECK_EQ(image->size(), hw * depth);
  CHECK_EQ(image->row(0).size(), hw);
  CHECK_EQ(image->col(0).size(), depth);

  std::vector<int> infinite_pixels;
  // compute min and max ignoring nonfinite pixels
  float image_min = std::numeric_limits<float>::infinity();
  float image_max = -image_min;
  for (int i = 0; i < hw; i++) {
    bool finite = true;
    for (int j = 0; j < depth; j++) {
      // if infinite, skip this pixel
      if (!std::isfinite(values(j, i))) {
        infinite_pixels.emplace_back(i);
        finite = false;
        break;
      }
    }
    if (finite) {
      for (int j = 0; j < depth; j++) {
        float v = values(j, i);
        image_min = std::min(image_min, v);
        image_max = std::max(image_max, v);
      }
    }
  }

  // Pick an affine transform into uint8
  const float kZeroThreshold = 1e-6;
  float scale, offset;
  if (image_min < 0) {
    float max_val = std::max(std::abs(image_min), image_max);
    scale = (max_val < kZeroThreshold ? 0.0f : 127.0f) / max_val;
  } else {
    scale = (image_max < kZeroThreshold ? 0.0f : 255.0f) / image_max;
    offset = 0.0f;
  }

  // Transform image, turning nonfinite values to bad_color
  for (int i = 0; i < depth; i++) {
    auto tmp = scale * values.row(i).array() + offset;
    image->row(i) = tmp.cast<uint8_t>();
  }

  for (int pixel : infinite_pixels) {
    for (int i = 0; i < depth; i++) {
      // TODO(ChunweiYan) use some highlight color to represent infinite pixels.
      (*image)(pixel, i) = (uint8_t)0;
    }
  }
}

}  // namespace visualdl

#endif
