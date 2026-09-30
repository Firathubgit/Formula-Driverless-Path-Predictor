#include "race_samples.hpp"
#include <QFile>
#include <QResource>
#include <QtEndian>
#include <stdexcept>

static void initialize_recordings() { Q_INIT_RESOURCE(race_recordings); }
namespace race_sound {
SampleLoop decode_wave(const QByteArray& bytes,double rms) {
    const auto u16=[&](qsizetype at){return qFromLittleEndian<quint16>(bytes.constData()+at);};
    const auto u32=[&](qsizetype at){return qFromLittleEndian<quint32>(bytes.constData()+at);};
    if(bytes.size()<44 || bytes.size()>24000000 || bytes.first(4)!="RIFF" || bytes.mid(8,4)!="WAVE" ||
       quint64(u32(4))+8!=quint64(bytes.size())) throw std::invalid_argument("Invalid WAV container");
    int channels=0,rate=0; QByteArray pcm;
    for(qsizetype at=12;at+8<=bytes.size();) {
        const auto length=quint64(u32(at+4));
        if(length>quint64(bytes.size()-at-8)) throw std::invalid_argument("Truncated WAV chunk");
        const auto tag=bytes.mid(at,4); at+=8;
        if(tag=="fmt ") {
            if(length<16 || u16(at)!=1 || u16(at+14)!=16) throw std::invalid_argument("Expected PCM16 WAV");
            channels=u16(at+2); rate=int(u32(at+4));
            if(channels<1 || channels>2 || rate<22050 || rate>192000 || u16(at+12)!=channels*2 ||
               u32(at+8)!=quint32(rate*channels*2)) throw std::invalid_argument("Unsupported WAV format");
        } else if(tag=="data") pcm=bytes.mid(at,qsizetype(length));
        at+=qsizetype(length+(length&1));
    }
    if(channels==0 || pcm.isEmpty() || pcm.size()%(channels*2)) throw std::invalid_argument("Incomplete WAV");
    std::vector<float> mono(pcm.size()/(channels*2));
    for(std::size_t i=0;i<mono.size();++i) {
        double sample=0;
        for(int ch=0;ch<channels;++ch) sample+=qFromLittleEndian<qint16>(pcm.constData()+(i*channels+ch)*2)/32768.0;
        mono[i]=static_cast<float>(sample/channels);
    }
    return SampleLoop(std::move(mono),rate,rms);
}
std::shared_ptr<const Bank> recorded_bank() {
    static const auto bank=[] {
        initialize_recordings();
        const auto read=[](const char* name,double rms) {
            QFile file(name);
            if(!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Recorded engine assets are missing");
            return decode_wave(file.readAll(),rms);
        };
        return std::make_shared<const Bank>(Bank{read(":/audio/f1v10.wav",0.30),read(":/audio/skid_tyres.wav",0.24)});
    }();
    return bank;
}
}
