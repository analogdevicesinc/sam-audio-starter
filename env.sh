#Set your CCES version and root here
CCES_VERSION=2.12.0
DRIVE=c

#Set both options for legacy and new path versions
CCES_PATH_LEGACY_SHARC="${DRIVE}:\Analog Devices\CrossCore Embedded Studio ${CCES_VERSION}"
CCES_PATH_LEGACY_ARM="${DRIVE}:\Analog Devices\CrossCore Embedded Studio ${CCES_VERSION}\ARM\arm-none-eabi\bin"
CCES_PATH_SHARC="${DRIVE}:/Analog/cces/${CCES_VERSION}"
CCES_PATH_ARM="${DRIVE}:/Analog/cces/${CCES_VERSION}/ARM/arm-none-eabi/bin"

#Verify path
if [[ -d "$CCES_PATH_LEGACY_SHARC" && -d "$CCES_PATH_LEGACY_ARM" ]]; then
    export PATH=$PATH:"/${DRIVE}/Analog Devices/CrossCore Embedded Studio ${CCES_VERSION}":"/${DRIVE}/Analog Devices/CrossCore Embedded Studio ${CCES_VERSION}/ARM/arm-none-eabi/bin"
    #echo $PATH
    echo Using legacy CCES path: "Version: "$CCES_VERSION" on drive "$DRIVE":"
elif [[ -d "$CCES_PATH_SHARC" && -d "$CCES_PATH_ARM" ]]; then
    export PATH=$PATH:"/${DRIVE}/Analog/cces/${CCES_VERSION}":"/${DRIVE}/Analog/cces/${CCES_VERSION}/ARM/arm-none-eabi/bin"
    #echo $PATH
    echo Using new CCES path: "Version: "$CCES_VERSION" on drive "$DRIVE":"
else
    echo Could not locate CCES Version. Please double check path. 
fi
