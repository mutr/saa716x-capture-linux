CONFIG_SAA716X_CORE=m \
	CONFIG_SAA716X_CAPTURE=m \
	CONFIG_MST3367=m \
	CONFIG_ECHDCAP_RX=m \
	make -C /lib/modules/$(uname -r)/build/ M=$(pwd) modules
#CONFIG_SAA716X_CORE=m CONFIG_SAA716X_CAPTURE=m make -C /lib/modules/$(uname -r)/build/ M=`pwd` modules

