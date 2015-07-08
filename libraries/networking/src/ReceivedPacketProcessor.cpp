//
//  ReceivedPacketProcessor.cpp
//  libraries/networking/src
//
//  Created by Brad Hefta-Gaub on 8/12/13.
//  Copyright 2013 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "NodeList.h"
#include "ReceivedPacketProcessor.h"
#include "SharedUtil.h"

void ReceivedPacketProcessor::terminating() {
    _hasPackets.wakeAll();
}

void ReceivedPacketProcessor::queueReceivedPacket(const SharedNodePointer& sendingNode, const QByteArray& packet) {
    // Make sure our Node and NodeList knows we've heard from this node.
    sendingNode->setLastHeardMicrostamp(usecTimestampNow());

    // TODO: fix the NodePacketPair once we've figured out receive API
    NodePacketPair networkPacket(sendingNode, NLPacket::create(PacketType::OctreeStats));

    lock();
    _packets.push_back(networkPacket);
    _nodePacketCounts[sendingNode->getUUID()]++;
    unlock();

    // Make sure to wake our actual processing thread because we now have packets for it to process.
    _hasPackets.wakeAll();
}

bool ReceivedPacketProcessor::process() {

    if (_packets.size() == 0) {
        _waitingOnPacketsMutex.lock();
        _hasPackets.wait(&_waitingOnPacketsMutex, getMaxWait());
        _waitingOnPacketsMutex.unlock();
    }

    preProcess();
    if (!_packets.size()) {
        return isStillRunning();
    }

    lock();
    std::list<NodePacketPair> currentPackets;
    currentPackets.swap(_packets);
    unlock();

    foreach(auto& packetPair, currentPackets) {
        // TODO: Replace QByteArray() once NLPacket is coming through on receive side
        processPacket(packetPair.first, QByteArray());
        midProcess();
    }

    lock();
    foreach(auto& packetPair, currentPackets) {
        _nodePacketCounts[packetPair.first->getUUID()]--;
    }
    unlock();

    postProcess();
    return isStillRunning();  // keep running till they terminate us
}

void ReceivedPacketProcessor::nodeKilled(SharedNodePointer node) {
    lock();
    _nodePacketCounts.remove(node->getUUID());
    unlock();
}
