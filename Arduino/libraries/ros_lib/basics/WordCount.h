#ifndef _ROS_SERVICE_WordCount_h
#define _ROS_SERVICE_WordCount_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace basics
{

static const char WORDCOUNT[] = "basics/WordCount";

  class WordCountRequest : public ros::Msg
  {
    public:
      const char* words;

    WordCountRequest():
      words("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_words = strlen(this->words);
      memcpy(outbuffer + offset, &length_words, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->words, length_words);
      offset += length_words;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_words;
      memcpy(&length_words, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_words; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_words-1]=0;
      this->words = (char *)(inbuffer + offset-1);
      offset += length_words;
     return offset;
    }

    const char * getType(){ return WORDCOUNT; };
    const char * getMD5(){ return "6f897d3845272d18053a750c1cfb862a"; };

  };

  class WordCountResponse : public ros::Msg
  {
    public:
      uint32_t count;

    WordCountResponse():
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->count =  ((uint32_t) (*(inbuffer + offset)));
      this->count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return WORDCOUNT; };
    const char * getMD5(){ return "ac8b22eb02c1f433e0a55ee9aac59a18"; };

  };

  class WordCount {
    public:
    typedef WordCountRequest Request;
    typedef WordCountResponse Response;
  };

}
#endif
