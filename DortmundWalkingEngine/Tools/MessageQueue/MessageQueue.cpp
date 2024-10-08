/**
* @file MessageQueue.cpp
*
* Implementation of class MessageQueue and helper classes
* 
* @author Martin Lötzsch
*/

#include "MessageQueue.h"
#include "InMessage.h"
#include "OutMessage.h"
#include "Platform/GTAssert.h"

void MessageQueue::handleAllMessages(MessageHandler& handler)
{
  for(int i = 0; i < queue.numberOfMessages; ++i)
  {
    queue.setSelectedMessageForReading(i);
    in.config.reset();
    in.text.reset();
    handler.handleMessage(in);
  }
}

void MessageQueue::copyAllMessages(MessageQueue& other)
{
  char* dest = other.queue.reserve(queue.usedSize - MessageQueueBase::headerSize);
  if(dest)
  {
    memcpy(dest - MessageQueueBase::headerSize, queue.buf, queue.usedSize);
    other.queue.numberOfMessages += queue.numberOfMessages;
    other.queue.usedSize += queue.usedSize;
    other.queue.writePosition = 0;
  }
  else // Not all messages fit in there, so try step by step (some will be missing).
    for(int i = 0; i < queue.numberOfMessages; ++i)
      copyMessage(i, other);
}

void MessageQueue::moveAllMessages(MessageQueue& other)
{
  copyAllMessages(other);
  clear();
}

void MessageQueue::copyMessage(int message, MessageQueue& other)
{
  queue.setSelectedMessageForReading(message);
  other.out.bin.write(queue.getData(), queue.getMessageSize());
  other.out.finishMessage(queue.getMessageID());
}

void MessageQueue::write(Out& stream) const
{
  stream << queue.usedSize << queue.numberOfMessages;
  stream.write(queue.buf, queue.usedSize);
}

void MessageQueue::writeAppendableHeader(Out& stream) const
{
  stream << -1 << -1;
}

void MessageQueue::append(Out& stream) const
{
  stream.write(queue.buf, queue.usedSize);
}

void MessageQueue::append(In& stream)
{
  int usedSize,
      numberOfMessages;
  stream >> usedSize >> numberOfMessages;
  // Trying a direct copy. This is hacked, but fast.
  char* dest = numberOfMessages == -1 ? 0 : queue.reserve(usedSize - MessageQueueBase::headerSize);
  if(dest)
  {
    stream.read(dest - MessageQueueBase::headerSize, usedSize);
    queue.numberOfMessages += numberOfMessages;
    queue.usedSize += usedSize;
    queue.writePosition = 0;
  }
  else // Not all messages fit in there, so try step by step (some will be missing).
    for(int i = 0; (numberOfMessages == -1 && !stream.eof()) || i < numberOfMessages; ++i)
    {
      unsigned char id = 0;
      unsigned int size = 0;
      stream >> id;
      stream.read(&size, 3);
      char* dest = numberOfMessages != -1 || id < numOfDataMessageIDs ? queue.reserve(size) : 0;
      if(dest)
      {
        stream.read(dest, size);
        out.finishMessage(MessageID(id));
      }
      else
        stream.skip(size);
    }
}

Out& operator<<(Out& stream, const MessageQueue& messageQueue)
{
  messageQueue.write(stream);
  return stream;
}

In& operator>>(In& stream, MessageQueue& messageQueue)
{
  messageQueue.append(stream);
  return stream;
}

void operator>>(InMessage& message, MessageQueue& queue)
{
  queue.out.bin.write(message.getData(), message.getMessageSize());
  queue.out.finishMessage(message.getMessageID());
}
