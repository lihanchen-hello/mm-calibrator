# Generates up-to-date documentation.
# 
# The following may need to be entered into the console in order to run this script:
# chmod +x generate_docs.sh

doxygen doxytemplate
make -C latex
rm ../manual.html
ln -s ./documentation/html/index.html ../manual.html
rm ../manual.pdf
ln -s ./documentation/latex/refman.pdf ../manual.pdf
