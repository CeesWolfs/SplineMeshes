#include "SubFaceTree.hpp"

/*
* TODO a SubDivide Faces method, which divides subfaces between two parents
*/

SubFaceIterator SubFaceTree::begin(halfFace start_node) {
    assert(start_node.isSubdivided());
    auto node_index = toNodeIndex(start_node);
    while(nodes[node_index].lower_child.isSubdivided()) {
        node_index = toNodeIndex(nodes[node_index].lower_child);
    }
    return {*this, node_index, true};
}

SubFaceIterator SubFaceTree::end() {
    return {*this, (uint32_t)-1, false};
}

SubFaceIterator SubFaceTree::find(halfFace start_node, halfFace toFind, Vertex toFindmiddle)
{
    assert(start_node.isSubdivided());
    bool lower = false;
    auto child = start_node;
    while (child.isSubdivided())
    {
        float split = nodes[toNodeIndex(child)].split_coord;
        switch (nodes[toNodeIndex(child)].split_axis)
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
        }
        start_node = child;
        child = lower ? nodes[toNodeIndex(start_node)].lower_child : nodes[toNodeIndex(start_node)].top_child;
    }
    assert(child == toFind);
    return SubFaceIterator(*this, toNodeIndex(start_node), lower);
}

/*
* Called whenever the twin of a halfFace is split in two
* Splits the twin of a halfFace, either starting a new tree in the subfacetree datastructure,
*  or first finding and splitting a subface already in a tree
* Depends on the halfFace lower already in the tree
*  returns the head of the tree
*/
halfFace SubFaceTree::splitHalfFace(const halfFace start_node, const halfFace twin, const Axis split_axis ,const Vertex split_point,const halfFace lower,const halfFace higher)
{
    float split;
    switch (split_axis)
    {
    case Axis::x:
        split = split_point.x;
        break;
    case Axis::y:
        split = split_point.y;
        break;
    case Axis::z:
        split = split_point.z;
        break;
    }
    
    uint32_t new_index = nodes.size();
    // Already a tree node
    if (start_node.isSubdivided())
    {
        const auto ref = find(start_node, lower, split_point); 
        const Node node = {ref.toIndex(), split, split_axis, lower, higher};
        nodes.push_back(node);
        // Create a link to the new node
        *ref = halfFace(new_index, 6);
        // Return just the head of the tree
        return start_node;   
    }
    // Create a new subfacetree
    else {
        const Node node = {twin, split, split_axis, lower, higher};
        nodes.push_back(node);
        // return new head
        return halfFace(new_index, 6);   
    }
    
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
