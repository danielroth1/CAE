#ifndef UISCENEGRAPHMANAGER_H
#define UISCENEGRAPHMANAGER_H

#include <QTreeWidget>
#include <data_structures/BidirectionalMap.h>
#include <map>
#include <scene/scene_graph/SGCore.h>

class UISGBidirectionalMap : public BidirectionalMap<QTreeWidgetItem*, SGNode*>
{
public:
    UISGBidirectionalMap();

};

#endif // UISCENEGRAPHMANAGER_H
