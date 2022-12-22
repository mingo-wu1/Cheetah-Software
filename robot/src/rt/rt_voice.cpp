/*!
 * @file rt_voice.cpp
 * @brief voice drivers
 * @author hanyuanqiang
 */

#include "rt/rt_voice.h"

RtVoice::RtVoice(PeriodicTaskManager *pTaskManager)
    : _pTaskManager(pTaskManager),
    _logger("RtVoice"),
    _voiceInitState(false)
{
    (void)_pTaskManager;
    RtVoiceVolume("100");
}

void RtVoice::RtVoiceVolume(std::string volume)
{
    int result = -1;
    std::string strResult(1024, '\0');
    std::string strCmd = "amixer -c 1 set PCM ";
    strCmd += volume;
    strCmd += "%";
    strCmd += " 2>&1";

    FILE *stream = popen(strCmd.c_str(), "r");

    if (NULL != stream)
    {
        size_t len = fread((char *)strResult.c_str(), 1, strResult.size(), stream);
        (void )len;
        
        result = pclose(stream);
    }
    
    if ((-1 == result) || (1 != WIFEXITED(result)) || (0 != WEXITSTATUS(result)))
    {
        QUADRUPED_ERROR(_logger, "RtVoiceVolume: %s", strResult.c_str());
    }
    else
    {
        _voiceInitState = true;
        QUADRUPED_INFO(_logger, "RtVoiceVolume: %s", strResult.c_str());
    }
}

void RtVoice::RtVoicePlay(std::string name, float period)
{
    _strPlayCmd = "sudo -u '#1000' XDG_RUNTIME_DIR=/run/user/1000 "
        "aplay -N -i -D plughw:CARD=CD002AUDIO,DEV=0 ../resources/sound/";
    _strPlayCmd += name;
    _strPlayCmd += " 2>&1";

    if (false == _voiceInitState)
    {
        QUADRUPED_ERROR(_logger, "RtVoice init failed, Cannot play sound");
        return;
    }

    RtVoiceStop();
    if (nullptr != _pTaskManager)
    {
        _periodFunc = std::make_shared<PeriodicMemberFunction<RtVoice>>(
            _pTaskManager, period, "voice_task", &RtVoice::_runPlay, this);
        _periodFunc->start();
    }
}

void RtVoice::RtVoiceStop()
{
    if (0 != _periodFunc.use_count())
    {
        _periodFunc->stop();
    }
}

void RtVoice::_runPlay()
{
    int result = -1;
    std::string strResult(1024, '\0');

    FILE *stream = popen(_strPlayCmd.c_str(), "r");

    if (NULL != stream)
    {
        size_t len = fread((char *)strResult.c_str(), 1, strResult.size(), stream);
        (void )len;
        
        result = pclose(stream);
    }
    
    if ((-1 == result) || (1 != WIFEXITED(result)) || (0 != WEXITSTATUS(result)))
    {
        QUADRUPED_ERROR(_logger, "_runPlay: %s", strResult.c_str());
    }
    else
    {
        QUADRUPED_INFO(_logger, "_runPlay: %s", strResult.c_str());
    }
}