#include "Primitive.h"
#include "threading.hpp"

struct OctTree : IntersectionAccelerator {
	struct Node {
		BBox box;
		Node *children[8] = {nullptr, };
		std::vector<Intersectable*> primitives;
		bool isLeaf() const {
			return children[0] == nullptr;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	int depth = 0;
	int leafSize = 0;
	int nodes = 0;
	int MAX_DEPTH = 35;
	int MIN_PRIMITIVES = 10;

	void clear(Node *n) {
		if (!n) {
			return;
		}

		for (int c = 0; c < 8; c++) {
			clear(n->children[c]);
			delete n->children[c];
		}
	}

	void clear() {
		clear(root);
		allPrimitives.clear();
	}

	void addPrimitive(Intersectable* prim) override {
		allPrimitives.push_back(prim);
	}

	void build(Node *n, int currentDepth = 0) {
		if (currentDepth >= MAX_DEPTH || n->primitives.size() <= MIN_PRIMITIVES) {
			leafSize = std::max(leafSize, int(n->primitives.size()));
			return;
		}

		depth = std::max(depth, currentDepth);
		BBox childBoxes[8];
		n->box.octSplit(childBoxes);

		for (int c = 0; c < 8; c++) {
			Node *& child = n->children[c];
			child = new Node;
			nodes++;
			memset(child->children, 0, sizeof(child->children));
			child->box = childBoxes[c];
			for (int r = 0; r < n->primitives.size(); r++) {
				if (n->primitives[r]->boxIntersect(child->box)) {
					child->primitives.push_back(n->primitives[r]);
				}
			}
			if (child->primitives.size() == n->primitives.size()) {
				build(child, MAX_DEPTH + 1);
			} else {
				build(child, currentDepth + 1);
			}
		}
		n->primitives.clear();
	}

	void build(Purpose purpose) override {
		const char *treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			MIN_PRIMITIVES = 4;
			treePurpose = " instances";
		} else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			MIN_PRIMITIVES = 20;
			treePurpose = " mesh";
		}

		if (root) {
			clear(root);
			delete root;
		}

		printf("Building%s oct tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		nodes = leafSize = depth = 0;
		root = new Node();
		root->primitives.swap(allPrimitives);
		for (int c = 0; c < root->primitives.size(); c++) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root);
		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);
	}

	bool intersect(Node *n, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			for (int c = 0; c < n->primitives.size(); c++) {
				if (n->primitives[c]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		} else {
			for (int c = 0; c < 8; c++) {
				if (n->children[c]->box.testIntersect(ray)) {
					if (intersect(n->children[c], ray, tMin, tMax, intersection)) {
						tMax = intersection.t;
						hasHit = true;
					}
				}
			}
		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		return intersect(root, ray, tMin, tMax, intersection);
	}

	bool isBuilt() const override {
		return root != nullptr;
	}

	~OctTree() override {
		clear();
	}
};

/// TODO: Implement one/both or any other acceleration structure and change makeDefaultAccelerator to create it
struct KDTree : IntersectionAccelerator {
	void addPrimitive(Intersectable *prim) override {}
	void clear() override {}
	void build(Purpose purpose) override {}
	bool isBuilt() const override { return false; }
	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override { return false; }
};


struct BVHTree : IntersectionAccelerator {

	struct Node {

		BBox box;

		Node() = default;

		Node* children[2] = { nullptr, nullptr};
		std::vector<Intersectable*> primitives;
		bool isLeaf() const {
			return children[0] == nullptr;
		}

		void setBranch(Node* n0, Node* n1) {
			children[0] = n0;
			children[1] = n1;
		}
		void setLeaf() {
			children[0] = nullptr;
			children[1] = nullptr;
		}

		void updateAabb()
		{
			box = BBox();
			if (isLeaf())
			{
				assert(primitives.size() > 0);
				box = primitives[0]->getBox();
				for (int i = 1; i < primitives.size(); ++i) {
					box.add(primitives[i]->getBox());
				}
			}
			else
			{
				box = children[0]->box;
				box.add(children[1]->box);
			}
		}
	};

	std::vector<Intersectable*> allPrimitives;
	std::vector<Node> nodesArr;

	inline Node* allocateNode() {
		nodesArr.reserve(1010010);//Hack to make it o work. Needs to be improved!
		nodesArr.push_back(Node());
		return &nodesArr.back();
	}

	Node* root = nullptr;
	int MAX_DEPTH = 35;

	void addPrimitive(Intersectable *prim) override {	

		allPrimitives.push_back(prim);
	}

	void clear() override {
		root = nullptr;
		nodesArr.clear();
		allPrimitives.clear();
	}

	
	void addPrimitiveToMacroCellNode(Intersectable& prim, Node** node, Node* parent)
	{
		BBox bBox = prim.getBox();
		if ((*node) != nullptr)
		{
			Node* subNode = allocateNode();
			subNode->primitives.push_back(&prim);
			subNode->setLeaf();
			subNode->box = bBox;
			
			insertNode(*subNode, node, parent, 1);
		}
		else
		{
			(*node) = allocateNode();
			(*node)->setLeaf();
			(*node)->primitives.push_back(&prim);
			(*node)->box = bBox;
		}
	}

	void insertNode(Node& node, Node** parent, Node* grandParent, int currentDepth)
	{
		assert(parent != nullptr);

		if ((*parent)->isLeaf())
		{
			if(currentDepth >= MAX_DEPTH && node.isLeaf())
			{
				for (int i = 0; i < node.primitives.size(); ++i)
				{
					(*parent)->primitives.push_back(node.primitives[i]);
				}
				node.primitives.clear();
			}
			else
			{
				Node* newParent = allocateNode();
				newParent->setBranch(&node, (*parent));
				if (grandParent != nullptr) {
					if (grandParent->children[0] == (*parent)) {
						grandParent->children[0] = newParent;
					}
					else if (grandParent->children[1] == (*parent)) {
						grandParent->children[1] = newParent;
					}
				}
				(*parent) = newParent;
			}
		}
		else
		{
			assert((*parent)->children[0] != nullptr && (*parent)->children[1] != nullptr);

			(*parent)->children[0]->updateAabb();
			(*parent)->children[1]->updateAabb();
			const BBox& childBox0 = (*parent)->children[0]->box;
			const BBox& childBox1 = (*parent)->children[1]->box;
			BBox combinedBox0 = childBox0;//(box0 + node.box);
			BBox combinedBox1 = childBox1;//(box1 + node.box);
			combinedBox0.add(node.box);
			combinedBox1.add(node.box);
			const float volumeDiff0 = combinedBox0.volume() - childBox0.volume();
			const float volumeDiff1 = combinedBox1.volume() - childBox1.volume();

			//insert to the child that gives less volume increase
			if (volumeDiff0 < volumeDiff1) insertNode(node, &(*parent)->children[0], (*parent), currentDepth+1);
			else insertNode(node, &(*parent)->children[1], (*parent), currentDepth+1);
		}
		(*parent)->updateAabb();
	}

	void build(Purpose purpose) override {
		const char* treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			treePurpose = " instances";
		}
		else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			treePurpose = " mesh";
		}

		printf("Building%s BVH tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		root = nullptr;

		const int NUM_MACRO_CELLS = 4096;

		int macroCellPrimitivCounts[NUM_MACRO_CELLS];
		Node* nodeObjectRefs[NUM_MACRO_CELLS];

		for (int i = 0; i < NUM_MACRO_CELLS; ++i) {
			macroCellPrimitivCounts[i] = 0;
			nodeObjectRefs[i] = nullptr;
		}

		BBox sceneBBox;
		for (int c = 0; c < allPrimitives.size(); ++c) {
			allPrimitives[c]->expandBox(sceneBBox);
		}

		vec3 rootBoxEdgesInverted = (sceneBBox.max - sceneBBox.min).inverted();
		for (int c = 0; c < allPrimitives.size(); ++c) {
			
			assert(allPrimitives[c] != nullptr);
			Intersectable& prim = *allPrimitives[c];
			vec3 primitivePosition = prim.getBox().center();
			vec3 coordinates = ((primitivePosition - sceneBBox.min) * rootBoxEdgesInverted) * 1024;

			uint32_t cellIndex = EncodeMorton3(coordinates);
			uint32_t macroCellIndex = cellIndex >> 18 & ~(-NUM_MACRO_CELLS);

			assert(macroCellIndex >= 0 && macroCellIndex < NUM_MACRO_CELLS);
			addPrimitiveToMacroCellNode(prim, &nodeObjectRefs[macroCellIndex], nullptr);
			macroCellPrimitivCounts[macroCellIndex]++;
		}

		for (int i = 0; i < NUM_MACRO_CELLS; ++i) {
			if (nodeObjectRefs[i] != nullptr) {
				if (root == nullptr) {
					root = nodeObjectRefs[i];
				}
				else
				{
					insertNode(*nodeObjectRefs[i], &root, nullptr, 1);
				}
			}
		}

		//TODO: Run bottom up, and combine nodes to minimize intersection cost, using surface area checks, and keeping track of total intersection cost estimates for individual nodes.

		float averNumPrimPerMacroCell = 0.0f;
		int maxNumPrimPerMacroCell = 0;
		int minNumPrimPerMacroCell = 0;
		for (int i = 0; i < NUM_MACRO_CELLS; ++i) {
			averNumPrimPerMacroCell += macroCellPrimitivCounts[i];
			if (macroCellPrimitivCounts[i] > maxNumPrimPerMacroCell) {
				maxNumPrimPerMacroCell = macroCellPrimitivCounts[i];
			}
			if (macroCellPrimitivCounts[i] < minNumPrimPerMacroCell) {
				minNumPrimPerMacroCell = macroCellPrimitivCounts[i];
			}
		}
		averNumPrimPerMacroCell /= (float)NUM_MACRO_CELLS;

		printf(" done in %lldms, nodes %d,\n num of primitives per macro cell: (average: %.2f, max: %d, min: %d),\n", timer.toMs(timer.elapsedNs()), nodesArr.size(), averNumPrimPerMacroCell, maxNumPrimPerMacroCell, minNumPrimPerMacroCell);
	}
	
	bool intersect(Node* n, const Ray& ray, float tMin, float& tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			/*if (n->primitive != nullptr)
			{
				if (n->primitive->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}*/
			for (int c = 0; c < n->primitives.size(); c++) {
				if (n->primitives[c]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		}
		else {
			for (int c = 0; c < 2; c++) {
				if (n->children[c]->box.testIntersect(ray)) {
					if (intersect(n->children[c], ray, tMin, tMax, intersection)) {
						tMax = intersection.t;
						hasHit = true;
					}
				}
			}
		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {

		return intersect(root, ray, tMin, tMax, intersection);
	}

	bool isBuilt() const override {
		return root != nullptr;
	}

	~BVHTree() override {
		clear();
	}
};

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//return AcceleratorPtr(new KDTree());
	return AcceleratorPtr(new BVHTree());
	//return AcceleratorPtr(new OctTree());
}

