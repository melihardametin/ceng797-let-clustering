import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

namespace src::letclustering;

// Rol bilgisini göndermek için enum
enum LetRole {
    ROLE_UNDEFINED = 0;
    ROLE_CH = 1;
    ROLE_MEMBER = 2;
}

class LetHello extends inet::FieldsChunk {
    int senderId;
    double currentWeight;
    int role;
    int currentLeaderId; 
    
    double positionX;
    double positionY;
    double speed;
    double angle;
}