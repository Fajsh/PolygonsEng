#include "QuadTree.h"
#include <queue>
int QuadTreeNode::maxDepth = 5;
int QuadTreeNode::maxObjectsPerNode = 15;

bool QuadTreeNode::IsLeaf()
{
	return children.size() == 0;
}

int QuadTreeNode::NumObj()
{
	Reset();
	int objectCount = contents.size();
	for (int i = 0, size = contents.size(); i < size; ++i)
	{
		contents[i]->flag = true;
	}
	std::queue<QuadTreeNode*>process;
	process.push(this);
	while (process.size() > 0)
	{
		QuadTreeNode* processing = process.back();
		if (!processing->IsLeaf())
		{
			for (int i = 0, size = processing->children.size(); i < size; ++i)
			{
				process.push(&processing->children[i]);
			}
		}
		else
		{
			for(int i=0, size = processing->contents.size();i<size;++i)
			{
				if (!processing->contents[i]->flag)
				{
					objectCount += 1;
					processing->contents[i]->flag = true;
				}
			}
		}
		process.pop();
	}
	Reset();
	return objectCount;
}

void QuadTreeNode::Insert(QuadTreeData& data)
{
	if (!RectangleRectangle(data.bounds, nodeBounds))
	{
		return;
	}
	if (IsLeaf()&&contents.size()+1>maxObjectsPerNode)
	{
		Split();
	}
	if (IsLeaf())
	{
		contents.push_back(&data);
	}
	else
	{
		for (int i = 0, size = children.size(); i < size; ++i)
		{
			children[i].Insert(data);
		}
	}

}

void QuadTreeNode::Remove(QuadTreeData& data)
{
	if (IsLeaf())
	{
		int removeIndex = -1;
		for (int i = 0, size = contents.size(); i < size; ++i)
		{
			if (contents[i]->object == data.object)
			{
				removeIndex = i;
				break;
			}
		}
		if (removeIndex != -1)
		{
			contents.erase(contents.begin() + 1);
		}
	}
	else
	{
		for (int i = 0, size = children.size(); i < size; ++i)
		{
			children[i].Remove(data);
		}
	}
	Shake();
}

void QuadTreeNode::Update(QuadTreeData& data)
{
	Remove(data);
	Insert(data);
}

void QuadTreeNode::Shake()
{
	if (IsLeaf())
	{
		int numObjects = NumObj();
		if (numObjects == 0)
		{
			children.clear();
		}
		else if (numObjects < maxObjectsPerNode)
		{
			std::queue<QuadTreeNode*> process;
			process.push(this);
			while (process.size() > 0)
			{
				QuadTreeNode* processing = process.back();
				if (processing->IsLeaf())
				{
					for (int i = 0, size = processing->children.size(); i < size; ++i)
					{
						process.push(&processing->children[i]);
					}
				}
				else
				{
					contents.insert(contents.end(), processing->contents.begin(), processing->contents.end());
				}
				process.pop();
			}
			children.clear();
		}
	}
}

void QuadTreeNode::Split()
{
	if (curDepth + 1 >= maxDepth)
	{
		return;
	}
	vec2 min = GetMin(nodeBounds);
	vec2 max = GetMax(nodeBounds);
	vec2 center = min + ((max - min) * 0.5f);
	Rectangle2D childAreas[] = {
		Rectangle2D(FromMinMax(vec2(min.x,min.y),vec2(center.x,center.y))),
		Rectangle2D(FromMinMax(vec2(center.x,min.y),vec2(max.x,center.y))),
		Rectangle2D(FromMinMax(vec2(center.x,center.y),vec2(max.x,max.y))),
		Rectangle2D(FromMinMax(vec2(min.x,center.y),vec2(center.x,max.y)))
	};
	for (int i = 0; i < 4; ++i)
	{
		children.push_back(QuadTreeNode(childAreas[i]));
		children[i].curDepth = curDepth + 1;
	}
	for (int i = 0, size = contents.size(); i < size; ++i)
	{
		children[i].Insert(*contents[i]);
	}
	contents.clear();
}

void QuadTreeNode::Reset()
{
	if (IsLeaf())
	{
		for (int i = 0, size = contents.size(); i < size; ++i)
		{
			contents[i]->flag = false;
		}
	}
	else
	{
		for (int i = 0, size = children.size(); i < size; ++i)
		{
			children[i].Reset();
		}
	}
}

vector<QuadTreeData*> QuadTreeNode::Query(const Rectangle2D& area)
{
	std::vector<QuadTreeData*>res;
	if (!RectangleRectangle(area, nodeBounds))return res;
	if (IsLeaf())
	{
		for (int i = 0, size = contents.size(); i < size; ++i)
		{
			if (RectangleRectangle(contents[i]->bounds, area))
			{
				res.push_back(contents[i]);
			}
		}
	}
	else
	{
		for (int i = 0, size = children.size(); i < size; ++i)
		{
			vector<QuadTreeData*> recurse = children[i].Query(area);
			if (recurse.size() > 0)
			{
				res.insert(res.end(), recurse.begin(), recurse.end());
			}
		}
	}
	return res;
}

