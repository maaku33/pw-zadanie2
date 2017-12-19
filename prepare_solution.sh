#!/bin/bash
rm -r logs
rm -r tests-1

cd ms386007
zip ms386007.zip {*.cpp,*.hpp,CMakeLists.txt,raport.pdf}
mv ms386007.zip ../sols/

cd ../verify
python3 verify.py
cd ..
