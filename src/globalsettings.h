#ifndef GLOBALSETTINGS_H
#define GLOBALSETTINGS_H

#include <QObject>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>

class GlobalSettings : public QObject
{
public:
    static std::shared_ptr<GlobalSettings> instance()
    {
        static  std::shared_ptr<GlobalSettings> instance {new GlobalSettings};
        return instance;
    }

    GlobalSettings(GlobalSettings const&) = delete;
    void operator=(GlobalSettings const&) = delete;

    int getBufferSize()  {return buffer_size;}
    void setBufferSize(int _buffer_size) { buffer_size=_buffer_size;}

    int getMinDepth()  {return min_depth;}
    void setMinDepth(int _min_depth) { min_depth=_min_depth;}

    int getMaxDepth()  {return max_depth;}
    void setMaxDepth(int _max_depth) { max_depth=_max_depth;}

private:
    GlobalSettings();

    int buffer_size = 8;
    int min_depth = 80;
    int max_depth = 1200;
};

#endif // GLOBALSETTINGS_H
