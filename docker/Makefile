.PHONY: build build-devel run run-devel

OCI="podman"

image_namespace=localhost
image_name=virelex
image_tag=latest

build_image_tag=$(image_namespace)/$(image_name):$(image_tag)
devel_image_tag=$(image_namespace)/$(image_name)_devel:$(image_tag)

containerfile=base.Containerfile

# Check OCI runtime for an initialized build base image
build_image_id=$(shell $(OCI) images -q $(build_image_tag) 2> /dev/null)

# If build base image is not initialized, add the full build to relevant prereqs
ifdef build_image_id
bootstrap_build=
else
bootstrap_build=build
endif

build:
	$(OCI) build \
		--tag  "$(build_image_tag)" \
		--file "$(containerfile)" \
		..

build-devel: $(bootstrap_build)
	@echo build_image_id $(build_image_id)
	$(OCI) build \
		--build-arg "base_image=$(build_image_tag)" \
		--tag  "$(devel_image_tag)" \
		--file "$(containerfile)" \
		..

run: $(bootstrap_build)
	$(OCI) \
	    run --rm --interactive --tty  \
	    \
	    --security-opt "label=type:container_runtime_t" \
	    \
	    --device "nvidia.com/gpu=all" \
	    -e "DISPLAY" \
	    -v "/tmp/.X11-unix:/tmp/.X11-unix:ro" \
	    -v "..:/ws/src:ro" \
	    \
	    $(build_image_tag)

run-devel: build-devel
	$(OCI) \
	    run --rm --interactive --tty  \
	    \
	    --security-opt "label=type:container_runtime_t" \
	    \
	    --device "nvidia.com/gpu=all" \
	    -e "DISPLAY" \
	    -v "/tmp/.X11-unix:/tmp/.X11-unix:ro" \
	    -v "..:/ws/src:ro" \
	    \
	    $(devel_image_tag)

