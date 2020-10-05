# Makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
PAPER         =
BUILDDIR      = build

# Internal variables.
PAPEROPT_a4     = -D latex_paper_size=a4
PAPEROPT_letter = -D latex_paper_size=letter
ALLSPHINXOPTS   = -d $(BUILDDIR)/doctrees $(PAPEROPT_$(PAPER)) $(SPHINXOPTS) .

.PHONY: help clean html latex

html: cmakeapi
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) ..
	@echo
	@echo "Build finished. The HTML pages are in .."

latex: cmakeapi
	$(SPHINXBUILD) -b latex $(ALLSPHINXOPTS) latex
	cd latex && make
	cp latex/api.pdf ..
	@echo
	@echo "Build finished. The pdf pages are in .."

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  html       to make standalone HTML files"

clean:
	-rm generated_cmake_api.rst
	-rm -rf build
	-rm -rf latex

cmakeapi:
	./generate_cmake_rst.py -o generated_cmake_api.rst  ../../cmake


