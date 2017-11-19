# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = python -msphinx
SPHINXPROJ    = openvd
SOURCEDIR     = source
BUILDDIR      = build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

example:
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/TemplateSimple/TemplateSimple.m > source/exampleTemplateSimple.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/TemplateArticulated/TemplateArticulated.m > source/exampleTemplateArticulated.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/TemplateSimpleSimulink/TemplateSimpleSimulink.m > source/exampleTemplateSimpleSimulink.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/TemplateArticulatedSimulink/TemplateArticulatedSimulink.m > source/exampleTemplateArticulatedSimulink.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/SinusoidalSteering/SinusoidalSteering.m > source/exampleSinusoidalSteering.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/SkidPad/SkidPad.m > source/exampleSkidPad.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/SkidPad4DOF/SkidPad4DOF.m > source/exampleSkidPad4DOF.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/SteeringControl/SteeringControl.m > source/exampleSteeringControl.rst
	sed -e '/^%/!s/^/    &/' -e 's/^% /%/' -e 's/^%//' examples/KalmanFilter/KalmanFilter.m > source/exampleKalmanFilter.rst

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
