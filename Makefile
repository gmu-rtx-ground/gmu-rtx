ifdef CATKIN_WS
  $(info CATKIN_WS is defined as $(CATKIN_WS))
else
  $(error CATKIN_WS is not defined, please set it)
endif

SRC_BASE := $(HOME)/gmu-rtx/ROS
DST_BASE := $(CATKIN_WS)/src

.PHONY: links unlink

links:
	@echo "Creating symlinks for ROS packages..."
	@for dir in $(SRC_BASE)/*; do \
		name=$$(basename $$dir); \
		if [[ "$$name" = "udev_rules" || ! -d "$$dir" ]]; then \
			continue; \
		fi; \
		ln -s "$(SRC_BASE)/$$name" "$(DST_BASE)/$$name"; \
	done
	@echo "Symlinks can be removed using: $ make unlink"

unlink:
	@echo "Removing symlinks from $(DST_BASE)..."
	@for item in $(DST_BASE)/*; do \
		if [ -L "$$item" ]; then \
			unlink "$$item"; \
		fi; \
	done
