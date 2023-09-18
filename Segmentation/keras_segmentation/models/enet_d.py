from keras.models import *
from keras.layers import *
from keras import layers
from .config import IMAGE_ORDERING
from .model_utils import get_segmentation_model
from .enet_e import get_enet_encoder

# decoder
def de_bottleneck(encoder, output, upsample=False, reverse_module=False):
    internal = output // 4

    x = Conv2D(internal, (1, 1), use_bias=False, data_format=IMAGE_ORDERING)(encoder)
    x = BatchNormalization(momentum=0.1)(x)
    x = Activation('relu')(x)
    if not upsample:
        x = Conv2D(internal, (3, 3), padding='same', use_bias=True, data_format=IMAGE_ORDERING)(x)
    else:
        x = Conv2DTranspose(filters=internal, kernel_size=(3, 3), strides=(2, 2), padding='same', data_format=IMAGE_ORDERING)(x)
    x = BatchNormalization(momentum=0.1)(x)
    x = Activation('relu')(x)

    x = Conv2D(output, (1, 1), padding='same', use_bias=False, data_format=IMAGE_ORDERING)(x)

    other = encoder
    if encoder.get_shape()[-1] != output or upsample:
        other = Conv2D(output, (1, 1), padding='same', use_bias=False, data_format=IMAGE_ORDERING)(other)
        other = BatchNormalization(momentum=0.1)(other)
        if upsample and reverse_module is not False:
            other = UpSampling2D(size=(2, 2))(other)

    if upsample and reverse_module is False:
        decoder = x
    else:
        x = BatchNormalization(momentum=0.1)(x)
        decoder = layers.add([x, other])
        decoder = Activation('relu')(decoder)

    return decoder

def de_build(encoder, nc):
    enet = de_bottleneck(encoder, 64, upsample=True, reverse_module=True)  # bottleneck 4.0
    enet = de_bottleneck(enet, 64)  # bottleneck 4.1
    enet = de_bottleneck(enet, 64)  # bottleneck 4.2
    enet = de_bottleneck(enet, 16, upsample=True, reverse_module=True)  # bottleneck 5.0
    enet = de_bottleneck(enet, 16)  # bottleneck 5.1

    enet = Conv2DTranspose(filters=nc, kernel_size=(2, 2), strides=(2, 2), padding='same', data_format=IMAGE_ORDERING)(enet)
    #print(enet.shape)
    return enet

def _enet(n_classes, encoder, input_height=512, input_width=512):
    img_input, f = encoder(input_height=input_height,  input_width=input_width)
    o = de_build(f, n_classes)
    model = get_segmentation_model(img_input, o)
    return model

def enet(n_classes, input_height=512, input_width=512):
    model = _enet(n_classes, get_enet_encoder,  input_height=input_height, input_width=input_width)
    model.model_name = "enet"
    return model