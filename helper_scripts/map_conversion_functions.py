import numpy as np
import cv2
import copy


def color2label(input_labels, num_classes=15):
    input_label = copy.deepcopy(input_labels)
    _step = num_classes / 255.0

    # apply binning on label values -> denoise
    _label = np.round(input_label * _step).astype(int)

    # decode labels from image values
    _out = _label[:, :, 0] + _label[:, :, 1] * num_classes + _label[:, :, 2] * num_classes ** 2
    return _out


def label2color(labels, num_classes=15):
    label = copy.deepcopy(labels)
    _step = num_classes #/ 255.0
    _devision1 = label // (num_classes * num_classes)
    _residual1 = label % (num_classes * num_classes)

    # second devision step
    _devision2 = _residual1 // num_classes
    _residual2 = _residual1 % num_classes

    # check if numbers are in boundary
    if type(label) is not int:
        if (_devision1*_step > num_classes).any():
            _devision1 = num_classes
            print('WARNING: Label class exceeded limit of class ID! TSS sets highest label level value to 255')
    else:
        if (_devision1*_step > num_classes):
            _devision1 = num_classes
            print('WARNING: Label class exceeded limit of class ID! TSS sets highest label level value to 255')

    # calculate resulting BGR vec
    _bgr_value = np.asarray([_residual2*int((255.0/_step)), _devision2*int((255.0/_step)), _devision1*int((255.0/_step))])
    if len(_bgr_value.shape) > 1:
        cv2.merge(_bgr_value)
        _bgr_value = np.swapaxes(_bgr_value, 0, 1)
        _bgr_value = np.swapaxes(_bgr_value, 1, 2)

    # return RGB value
    return _bgr_value.astype("uint8")
