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

    int getBufferSize() {return buffer_size;}
    void setBufferSize(int _buffer_size) { buffer_size=_buffer_size;}

private:
    GlobalSettings();

    int buffer_size=14;

};

#endif // GLOBALSETTINGS_H
