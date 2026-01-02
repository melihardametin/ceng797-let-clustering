import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

namespace src::letclustering;

enum LetRole {
    ROLE_UNDEFINED = 0;
    ROLE_CH = 1;
    ROLE_MEMBER = 2;
    ROLE_GATEWAY = 3;
}

// Kümeleme Mesajı
class LetHello extends inet::FieldsChunk {
    int senderId;
    double currentWeight;
    int role;
    int currentLeaderId; 
    
    // LET Hesabı için gerekli konum verileri
    double positionX;
    double positionY;
    double speed;
    double angle;
}

// Veri Mesajı
class LetData extends inet::FieldsChunk {
    int srcId;          
    int destId;
    unsigned long seqId;
    int hopCount;       
    simtime_t creationTime;
}