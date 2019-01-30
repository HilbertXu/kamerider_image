
#ifndef TIMESTRAMP_H
#define TIMESTRAMP_H
 
#include <stdio.h>
#include <iostream>
 
#ifndef _WIN32
#include <sys/time.h>
#else
#include <windows.h>
#endif
 
using namespace std;
 
#define TIME_DEBUG(str) \
	cout <<  "[" << __DATE__ "] data:" << str << endl
 
#ifndef _WIN32
class timestramp{
public:
    timestramp()
    {
         gettimeofday(&tpstart,NULL);
    }
 
    ~timestramp()
    {
         gettimeofday(&tpend,NULL);
         //timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;//注意，秒的读数和微秒的读数都应计算在内
         timeuse = (1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec)/1000;
         printf("used time:%fms\n",timeuse);
    }
private:
     struct timeval tpstart,tpend;
     double timeuse;
};
 
#else
class timestramp{
private:
    LARGE_INTEGER m_litmp;
    LONGLONG QPart2;
    LONGLONG QPart1;
    double dfMinus, dfFreq, dfTim;
public:
    timestramp(){
        QueryPerformanceFrequency(&m_litmp);
        dfFreq = (double)m_litmp.QuadPart;
        QueryPerformanceCounter(&m_litmp);
        QPart1 = m_litmp.QuadPart;
    }
 
    ~timestramp(){
        QueryPerformanceCounter(&m_litmp);
        QPart2 = m_litmp.QuadPart;
        dfMinus = (double)(QPart2 - QPart1);
        dfTim = dfMinus / dfFreq * 1000;
 
        //显示时间
        std::string msg4 = "time:", msg3, msg5 = "ms";
		char strTime[20] = "";
		sprintf(strTime, "%.6lf", dfTim);
		msg3 = strTime;
		msg4 += msg3;
		msg4 += msg5;
		TIME_DEBUG(msg4.c_str());
    }
};
#endif
 
#endif // TIMESTRAMP_H
