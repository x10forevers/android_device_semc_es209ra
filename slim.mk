# Specify phone tech before including full_phone
$(call inherit-product, vendor/slim/config/common.mk)

# Release name
PRODUCT_RELEASE_NAME := es209ra

# Inherit some common Slim stuff.
$(call inherit-product, vendor/slim/config/common_full_phone.mk)

# Inherit Device Settings
$(call inherit-product, vendor/slim/config/common_semc.mk)

# Inherit device configuration
$(call inherit-product, device/semc/es209ra/full_es209ra.mk)

# Inherit torch settings
$(call inherit-product, vendor/slim/config/common_ledflash.mk)

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE := es209ra
PRODUCT_NAME := slim_es209ra
PRODUCT_BRAND := SEMC
PRODUCT_MODEL := Xperia-X10
PRODUCT_MANUFACTURER := Sony Ericsson

#Extra Device info
PRODUCT_PROPERTY_OVERRIDES += \
  ro.device.rear_cam=8MP \
	ro.device.screen_res=480x854

#Boot Animation
PRODUCT_COPY_FILES += \
	vendor/slim/prebuilt/hdpi/bootanimation.zip:system/media/bootanimation.zip

#copy 00check
PRODUCT_COPY_FILES += \
	vendor/slim/prebuilt/common/etc/init.d/00check:system/etc/init.d/00check

#Set build fingerprint / ID / Prduct Name ect.
PRODUCT_BUILD_PROP_OVERRIDES += PRODUCT_NAME=LT18i BUILD_FINGERPRINT="SEMC/LT18i_1254-2184/LT18i:4.0.4/4.1.B.0.431/UL5_3w:user/release-keys" PRIVATE_BUILD_DESC="LT18i-user 4.0.4 4.1.B.0.431 UL5_3w test-keys"
