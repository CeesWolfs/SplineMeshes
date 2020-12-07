#include "SubFaceTree.hpp"

SubFaceIterator<SubFaceTree> SubFaceTree::begin(halfFace start_node) {
    assert(start_node.isSubdivided());
    auto node_index = toNodeIndex(start_node);
    while(nodes[node_index].lower_child.isSubdivided()) {
        node_index = toNodeIndex(nodes[node_index].lower_child);
    }
    return {this, node_index, true};
}

SubFaceIterator<SubFaceTree> SubFaceTree::end() {
    return {this, (uint32_t)-1, false};
}

SubFaceIterator<const SubFaceTree> SubFaceTree::cbegin(halfFace start_node) const {
    assert(start_node.isSubdivided());
    auto node_index = toNodeIndex(start_node);
    while (nodes[node_index].lower_child.isSubdivided()) {
        node_index = toNodeIndex(nodes[node_index].lower_child);
    }
    return { this, node_index, true };
}

SubFaceIterator<const SubFaceTree>SubFaceTree::cend() const {
    return { this, (uint32_t)-1, false };
}

void SubFaceTree::updateSubTreeTwins(const halfFace head, const halfFace old_hf, const halfFace new_hf, const Vertex& split_point, std::vector<halfFace>& F2f)
{
    auto updateTwin = [&](halfFace twin) {
        if (twin.isSubdivided()) {
            auto it = this->find(twin, old_hf, split_point);
            *it = new_hf;
        }
        else {
            F2f[twin.getCuboid() * 6 + twin.getLocalId()] = new_hf;
        }
    };
    if (!head.isSubdivided()) {
        updateTwin(head);
        return;
    }
    for (auto it = cbegin(head); it != cend(); ++it)
    {
        updateTwin(*it);
    }
}

SubFaceIterator<SubFaceTree> SubFaceTree::find(halfFace start_node, const halfFace toFind, const Vertex& toFindmiddle)
{
    assert(start_node.isSubdivided());
    bool lower = false;
    auto child = start_node;
    while (child.isSubdivided())
    {
        float split = nodes[toNodeIndex(child)].split_coord;
        switch (nodes[toNodeIndex(child)].split_axis)
        {
            case Axis::x :
                lower = (eps + toFindmiddle.x <= split);
                break;
            case Axis::y :
                lower = (eps + toFindmiddle.y <= split);
                break;
            case Axis::z :
                lower = (eps + toFindmiddle.z <= split);
                break;
        }
        start_node = child;
        child = lower ? nodes[toNodeIndex(start_node)].lower_child : nodes[toNodeIndex(start_node)].top_child;
    }
    assert(child == toFind);
    return SubFaceIterator<SubFaceTree>(this, toNodeIndex(start_node), lower);
}

bool SubFaceTree::findVertexBorder(halfFace start_node, const Vertex& vertexToFind, const Axis splitAxis, halfFace& found) const
{
    if (!start_node.isSubdivided()) {
        throw "ERROR[SubFaceTree::findVertex]: start node is not subdivided!";
    }

    bool lower = false;
    bool vertexFound = false;
    auto child = start_node;

    while (child.isSubdivided())
    {
        
        const Node nodeToCheck = nodes[toNodeIndex(child)];
        const float split = nodeToCheck.split_coord;
        switch (nodeToCheck.split_axis)
        {
        case Axis::x:
            if (splitAxis == Axis::x)
            {
                // Use or, since if we have already found a split at the right level, we still have found the vertex
                vertexFound |= floatSame(vertexToFind.x, split);
            }
            lower = (vertexToFind.x <= split + eps);

            break;
        case Axis::y:
            if (splitAxis == Axis::y)
            {
                vertexFound |= floatSame(vertexToFind.y, split);
            }
            lower = (vertexToFind.y <= split + eps);
            break;
        case Axis::z:
            if (splitAxis == Axis::z)
            {
                vertexFound |= floatSame(vertexToFind.z, split);
            }
            lower = (vertexToFind.z <= split + eps);
            break;
        }
        start_node = child;
        child = lower ? nodes[toNodeIndex(start_node)].lower_child : nodes[toNodeIndex(start_node)].top_child;
    }
    found = child;
    return vertexFound;
}

bool SubFaceTree::findVertex(halfFace start_node, const Vertex& vertexToFind) const
{
    if (!start_node.isSubdivided()) {
        throw "ERROR[SubFaceTree::findVertex]: start node is not subdivided!";
    };

    bool lower = false;
    auto child = start_node;

    while (child.isSubdivided())
    {
        bool vertexFound = false;
        const Node nodeToCheck = nodes[toNodeIndex(child)];
        const float split = nodeToCheck.split_coord;

        switch (nodeToCheck.split_axis)
        {
        case Axis::x:
            lower = (eps + vertexToFind.x <= split);
            vertexFound = floatSame(vertexToFind.x, split);
            break;
        case Axis::y:
            lower = (eps + vertexToFind.y <= split);
            vertexFound = floatSame(vertexToFind.y, split);
            break;
        case Axis::z:
            lower = (eps + vertexToFind.z <= split);
            vertexFound = floatSame(vertexToFind.z, split);
            break;
        }

        if (vertexFound) {
            return true;
        }

        start_node = child;
        child = lower ? nodes[toNodeIndex(start_node)].lower_child : nodes[toNodeIndex(start_node)].top_child;
    }

    return false;
}

/*
* Called whenever the twin of a halfFace is split in two
* Splits the twin of a halfFace, either starting a new tree in the subfacetree datastructure,
*  or first finding and splitting a subface already in a tree
* Depends on the halfFace lower already in the tree
*  returns the head of the tree
*/
halfFace SubFaceTree::splitHalfFace(const halfFace start_node, const halfFace twin, const Axis split_axis ,const Vertex& split_point,const halfFace lower,const halfFace higher)
{
    float split{};
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

HalfFacePair SubFaceTree::splitTree(const halfFace tree_head, const Axis split_axis, const Vertex& split_point, const halfFace lower, const halfFace higher, std::vector<halfFace>& F2f)
{
    float split{};
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
    if (!tree_head.isSubdivided()) {
        assert(!tree_head.isBorder());
        // We need to split this halfFace
        F2f[tree_head.getCuboid() * 6 + tree_head.getLocalId()] = splitHalfFace(F2f[tree_head.getCuboid() * 6 + tree_head.getLocalId()], tree_head, split_axis, split_point, lower, higher);
        return { tree_head, tree_head };
    }
    halfFace lower_head{(uint32_t)-1}, top_head{(uint32_t)-1};
    auto node_idx = toNodeIndex(tree_head);
    const Node node = nodes[node_idx];
    if (node.split_axis == split_axis)
    {
        if (floatSame(split, node.split_coord))
        {
            // Split point is exactly on this nodes[node_idx]
            // It is no longer needed so remove it
            lower_head = node.lower_child;
            top_head = node.top_child;
            removeNode(toNodeIndex(tree_head), F2f);
            // Switch ownership of the right children of top head
            updateSubTreeTwins(top_head, lower, higher, split_point, F2f);
            return {lower_head, top_head};
        }
        if (split < nodes[node_idx].split_coord)
        {
            // Split point is on the lower side
            // The current node can be shifted to the top side
            top_head = tree_head;
            // Split the lower side
            auto ret = splitTree(node.lower_child, split_axis, split_point, lower, higher, F2f);
            updateParent(ret.second, halfFace(toNodeIndex(tree_head), 6));
            nodes[node_idx].lower_child = ret.second;
            // Switch ownership of the right children of top head
            updateSubTreeTwins(node.top_child, lower, higher, split_point, F2f);
            return {ret.first, top_head};
        }
        else
        {
            // Split point is on the higher side
            // The current node can be shifted to the lower
            lower_head = tree_head;
            // Split the top side
            auto ret = splitTree(node.top_child, split_axis, split_point, lower, higher, F2f);
            updateParent(ret.first, halfFace(toNodeIndex(tree_head), 7));
            nodes[node_idx].top_child = ret.first;
            return {lower_head, ret.second};
        }
    }
    else {
        // Both directions need to be split, copy the current node
        lower_head = tree_head;
        Node copy = node;
        // Split the lower child further possible bug if removes happen the node migh be in a different location
        // TOOD Check if this is likely and fix it
        auto lower_ret = splitTree(node.lower_child, split_axis, split_point, lower, higher, F2f);
        // Split the higher child further 
        auto higher_ret = splitTree(node.top_child, split_axis, split_point, lower, higher, F2f);
        nodes[node_idx].lower_child = lower_ret.first;
        copy.lower_child = lower_ret.second;
        top_head = halfFace(nodes.size(), 6);
        updateParent(lower_ret.second, halfFace(toNodeIndex(top_head), 6));
        nodes[node_idx].top_child = higher_ret.first;
        copy.top_child = higher_ret.second;
        updateParent(higher_ret.second, halfFace(toNodeIndex(top_head), 7)); 
        nodes.push_back(copy);
    }
    return {lower_head, top_head};
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
