/*!
 * @file rt_voice.h
 * @brief voice drivers
 * @author hanyuanqiang
 */

#ifndef _rt_voice
#define _rt_voice

#include "Utilities/PeriodicTask.h"
#include "Logger/Logger.h"

class RtVoice
{
public:
    explicit RtVoice(PeriodicTaskManager *pTaskManager);
    void RtVoiceVolume(std::string volume);
    void RtVoicePlay(std::string name, float period);
    void RtVoiceStop();
    ~RtVoice(){};
    
private:
    PeriodicTaskManager *_pTaskManager = nullptr;
    std::shared_ptr<PeriodicMemberFunction<RtVoice>> _periodFunc;
    std::string _strPlayCmd;
    BZL_QUADRUPED::Logger _logger;
    bool _voiceInitState;

    void _runPlay();
};

#endif