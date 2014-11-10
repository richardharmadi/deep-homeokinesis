/**
  Class to log data

  **/

#ifndef __LOGEABLE_H
#define __LOGEABLE_H

#include <sys/stat.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace std;
namespace lpzrobots{

  class Logeable{
    private:
    FILE* pFile;
    string header;
    bool writeHeader;
    string chooseFileName(const string fileName);
    void log(const string line);

    public:
    /**
     * If you want incremental files like: result0.txt result1.txt etc, do not include
     * extension in fileName string path/file_name default is txt
     * otherwise the fileName is used for the file name
     * const string fileName name of the file
     * const string header header of the file (only written if fileName file doesn't exists
     * bool incremental will generate a new file name based on the existence of fileName
    */
    Logeable(const string fileName, const string header = "", bool incremental = false);

    ~Logeable();

    FILE* getFile();
    void toLog(const string line);
  };

}


#endif
