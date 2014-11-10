#include "logeable.h"
#include <selforg/stl_adds.h>


namespace lpzrobots{
using namespace std;

  Logeable::Logeable(const string fileName, const string header, bool incremental) {
    string newName;
    this->header = header;

    if (incremental) newName = chooseFileName(fileName);
    else newName = fileName;

    struct stat stFileInfo;
    if ((stat(newName.c_str(), &stFileInfo) != 0) && (!header.empty())) 
      writeHeader = true;
    else 
      writeHeader = false;

    cout << "Log file name: " << newName << "\n";
    pFile = fopen(newName.c_str(), "a");
  }

  Logeable::~Logeable(){
    fclose(pFile);
  }

  string Logeable::chooseFileName(const string fileName){
    string newFileName(fileName.c_str());
    struct stat stFileInfo;
    int i = 0;
    bool fileExists = false;
    while( !fileExists ){
     string test( newFileName.c_str() + itos(i) + ".txt" );
      if (stat(test.c_str(), &stFileInfo) != 0) {
        fileExists = true;
      } else {
        i++;
      }
    }
    newFileName = newFileName + itos(i) + ".txt";
    return newFileName;
  }

  void Logeable::toLog(const string line) {
    if (writeHeader) {
      fprintf(pFile, "%s\n", header.c_str());
      writeHeader = false;
    }
    fprintf(pFile, "%s\n", line.c_str());
  }

  FILE* Logeable::getFile(){
    return pFile;
  }


}
