#LATEX = latex
LATEX = pdflatex
BIBTEX = bibtex
DVIPS = dvips
PS2PDF = ps2pdf
VIEWPDF = okular

BASE_NAME = referenceguide

OTHER_FILES = 

all: $(BASE_NAME).pdf 

$(BASE_NAME).pdf: $(BASE_NAME).tex $(OTHER_FILES) $(PNG_FILES) 
	$(LATEX) $< && $(LATEX) $< && $(LATEX) $<

$(BASE_NAME).dvi: $(BASE_NAME).tex
	$(LATEX) $< && $(LATEX) $< && $(LATEX) $<

$(BASE_NAME).ps: $(BASE_NAME).dvi
	$(DVIPS) -P pdf -f -t a4 $(BASE_NAME).dvi > $@

display: $(BASE_NAME).pdf
	$(VIEWPDF) $(BASE_NAME).pdf &

%.eps: %.dia
	cd `dirname $<` && dia `basename $<` -t eps

%.pdf: %.eps
	epstopdf $<

clean:
	rm -rf $(BASE_NAME).ps $(BASE_NAME).dvi $(BASE_NAME).log \
	      $(BASE_NAME).aux $(BASE_NAME).blg $(BASE_NAME).toc \
	      missfont.log $(BASE_NAME).bbl $(BASE_NAME).pdf \
				doxygen html latex $(BASE_NAME).out
