/*
 * This CUDA preprocessing implementation is based on the work from the YOLOv11-TensorRT repository:
 * https://github.com/spacewalk01/yolov11-tensorrt
 * 
 * The original implementation provides efficient CUDA-accelerated image preprocessing for YOLO object detection models,
 * including image resizing, normalization, BGR to RGB conversion, and channel reordering.
 * 
 * Original repository is licensed under the AGPL-3.0 License.
 */
#ifndef __MACROS_H
#define __MACROS_H

#ifdef API_EXPORTS
#if defined(_MSC_VER)
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
#else

#if defined(_MSC_VER)
#define API __declspec(dllimport)
#else
#define API
#endif
#endif  // API_EXPORTS

#if NV_TENSORRT_MAJOR >= 8
#define TRT_NOEXCEPT noexcept
#define TRT_CONST_ENQUEUE const
#else
#define TRT_NOEXCEPT
#define TRT_CONST_ENQUEUE
#endif

#endif  // __MACROS_H