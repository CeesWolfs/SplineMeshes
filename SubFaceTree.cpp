#include "SubFaceTree.hpp"

/*
* TODO a SubDivide Faces method, which divides subfaces between two parents
*/

SubFaceIterator SubFaceTree::begin(uint32_t start_node_index) {
    auto node_index = start_node_index;
    while(nodes[node_index].lower_child.isSubdivided()) {
        node_index = toNodeIndex(nodes[node_index].lower_child);
    }
    return {*this, node_index, true};
}

SubFaceIterator SubFaceTree::end() {
    return {*this, (uint32_t)-1, false};
}

auto SubFaceTree::find(halfFace start_node, halfFace toFind, Vertex toFindmiddle)
{
    assert(start_node.isSubdivided());
    while (start_node.isSubdivided())
    {
        float split = nodes[toNodeIndex(start_node)].split_coord;
        bool lower = false;
        switch (nodes[toNodeIndex(start_node)].split_axis)
        {
        case Axis::x:
            lower = (eps + toFindmiddle.x <= split);
            break;
        case Axis::y:
            lower = (eps + toFindmiddle.y <= split);
            break;
        case Axis::z:
            lower = (eps + toFindmiddle.z <= split);
            break;

            start_node = lower ? nodes[toNodeIndex(start_node)].lower_child : nodes[toNodeIndex(start_node)].top_child;
        }
        assert(start_node == toFind);
        return SubFaceIterator(*this, toNodeIndex(start_node), lower);
    }
    return end();
}
    void SubFaceTree::splitHalfFace(halfFace start_node, halfFace toSplit, Axis split_axis ,Vertex split_point, halfFace lower, halfFace higher)
{
    const auto ref = find(start_node, toSplit, split_point);
    float split;
    switch (split_axis)
    {
    case Axis::x:
        split = split_point.x;
        break;
    case Axis::y:
        split = split_point.x;
        break;
    case Axis::z:
        split = split_point.x;
        break;
    }
    // Push back the new node
    uint32_t new_index = nodes.size();
    const Node node = {*ref, split_point.x, split_axis, lower, higher};
    nodes.push_back(node);
    *ref = halfFace(new_index, 6);
}

void SubFaceTree::removeNode(uint32_t node_index, std::vector<halfFace>& F2f) 
{
    assert(nodes.size() > 0);
    if (node_index == nodes.size() - 1)
    {
        nodes.pop_back();
        return;
    }
    // Swap the removed node with the last node
    std::swap(nodes[node_index], *(nodes.end() - 1));
    // Decrease the vector size
    nodes.pop_back();
    auto& node = nodes[node_index];
    if (node.parent.isSubdivided()) {
        if (isHigher(node.parent))
            nodes[toNodeIndex(node.parent)].top_child = halfFace(node_index, 6);
        else 
            nodes[toNodeIndex(node.parent)].lower_child = halfFace(node_index, 6);
    }
    else {
        F2f[node.parent.id] = halfFace(node_index, 6);
    }
}

SubFaceTree::~SubFaceTree()
{
    
}
