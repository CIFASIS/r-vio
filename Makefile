help:
	@echo "help   -- print this help"
	@echo "shell  -- open shell in container"
	@echo "image  -- build Docker image"


image:
	docker build -t "rvio:ros-kinetic" .
shell:
	docker run -it --net=host rvio:ros-kinetic bash

.PHONY: help shell image

