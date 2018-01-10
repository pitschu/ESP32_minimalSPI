#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := miniSPI
##COMPONENT_DIRS = src_appl
EXTRA_CXXFLAGS = -fno-use-linker-plugin
EXTRA_COMPONENT_DIRS := ./main/TinyBasic
export EXTRA_COMPONENT_DIRS

# look at $PROJ/esp32.common.ld file. It must be named exactly as shown to be found by make instead of $IDF_PATH/components/esp32/ld/esp32.common.ld

include $(IDF_PATH)/make/project.mk

BUILD_BASE	= build
HTML_DIR	?= html_src
HTML_ZIP	?= html_zipped

ESP_HTML_TARGET	?= 0x200000
ESP_HTML_MAXSIZE ?= 0x1d0000


webpages.espzip:
	echo "$(PATH)" 
	echo $(BUILD_BASE)
	rm -rf ./$(BUILD_BASE)/$(HTML_ZIP)
	cp -r -f $(HTML_DIR) ./$(BUILD_BASE)/$(HTML_ZIP)
	cd ./$(HTML_DIR) ; find ./ \( -name "*.js" -o -name '*.css' -o -name '*.ico' \) -print -exec bash -c " eval gzip -c {} > ../$(BUILD_BASE)/$(HTML_ZIP)/{}" \;
	cd ./$(BUILD_BASE)/$(HTML_ZIP) ; find | ../../tools/mkespfsimage.exe -c 0 > ../../build/webpages.espzip; cd ../..
	ls -l ./build/webpages.espzip
	
htmlflash: webpages.espzip
	if [ $$(stat -c '%s' ./build/webpages.espzip) -gt $$(( $(ESP_HTML_MAXSIZE) )) ]; then echo "webpages.espzip too big!"; false; fi
	if [ $$(stat -c '%s' ./build/webpages.espzip) -lt $$(( 0x40 )) ]; then echo "webpages.espzip too SMALL!"; false; fi
	$(ESPTOOLPY_SERIAL) write_flash $(ESP_HTML_TARGET) ./build/webpages.espzip


objdump: $(APP_ELF)
	xtensa-lx106-elf-size -A -x $(APP_ELF) 
	xtensa-lx106-elf-objdump --source --all-headers --demangle --line-numbers --wide $(APP_ELF) >linker.lst
	-@echo ' '


