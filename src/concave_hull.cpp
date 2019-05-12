#include "concave_hull.h"
#include "Point.H"

//#include <QVector>


/*
Point::Point()
{
    x = 0;
    y = 0;
}

Point::Point(double _x, double _y)
{
    x = _x;
    y = _y;
}

Point & Point::operator=(const Point & other)
{
    x = other.x;
    y = other.y;
    return *this;
}

Point Point::operator+(const Point & other) const
{
    return Point(x + other.x, y + other.y);
}

Point Point::operator-(const Point & other) const
{
    return Point(x - other.x, y - other.y);
}

Point Point::operator*(double k) const
{
    return Point(x * k, y * k);
}

Point Point::operator/(double k) const
{
    return Point(x / k, y / k);
}

bool Point::operator==(const Point & other) const
{
    return x == other.x && y == other.y;
}
*/

double DotProduct(Point p, Point other)
{
    return p.X() * other.X() + p.Y() * other.Y();
}

double DistanceSquared(Point & p, Point & to)
{
    return (double)((to.X() - p.X()) * (to.X() - p.X()) + (to.Y() - p.Y()) * (to.Y() - p.Y()));
}

double Distance(Point & p, Point & to)
{
    return sqrt(DistanceSquared(p,to));
}


double Distance(Point & p, Point & segmentStart, Point & segmentEnd)
{
    const double l2 = DistanceSquared(segmentStart, segmentEnd);
    if (l2 == 0.0) {
        return Distance(p, segmentStart);   // v == w case
    }

    // Consider the line extending the segment, parameterized as v + t (w - v)
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    double t = DotProduct((p - segmentStart), (segmentEnd - segmentStart)) / l2;
    if (t < 0.0) {
        return Distance(p, segmentStart); // Beyond the 'v' end of the segment
    } else if (t > 1.0) {
        return Distance(p, segmentEnd);   // Beyond the 'w' end of the segment
    }

    // Projection falls on the segment
    Point projection; //= segmentStart + (segmentEnd - segmentStart) * t;
    projection.set(segmentStart.X() + (segmentEnd.X() - segmentStart.X())* t, segmentStart.Y() + (segmentEnd.Y() - segmentStart.Y())* t);
    return Distance(p, projection);
}

/*double DecisionDistance(const QList<Point> & points) const
{
    Point result = points[0];
    double dst = Distance(points[0]);
    for (int i = 1; i < points.size(); i++) 
    {
        Point cur = points[i];
        double curDistance = Distance(cur);
        if (curDistance < dst) {
            result = cur;
            dst = curDistance;
        }
    }
    return dst;
}*/



double IsLeft(Point p0, Point p1, Point p2)
{
    return (p1.X() - p0.X()) * (p2.Y() - p0.Y()) - (p2.X() - p0.X()) * (p1.Y() - p0.Y());
}

bool IsPointInsidePolygon(Point v, vector<Point> & polygon)
{
    bool result = false;
    int j = polygon.size() - 1;
    for (int i = 0; i < polygon.size(); i++)
    {
        if ((polygon[i].Y() < v.Y() && polygon[j].Y() > v.Y()) || (polygon[j].Y() < v.Y() && polygon[i].Y() > v.Y()))
        {
            if (polygon[i].X() + (v.Y() - polygon[i].Y()) / (polygon[j].Y() - polygon[i].Y()) * (polygon[j].X() - polygon[i].X()) < v.X())
            {
                result = !result;
            }
        }
        j = i;
    }
    return result;
}

bool CheckEdgeIntersection(Point & p0, Point & p1, Point & p2, Point & p3)
{
    double s1_x = p1.X() - p0.X();
    double s1_y = p1.Y() - p0.Y();
    double s2_x = p3.X() - p2.X();
    double s2_y = p3.Y() - p2.Y();
    double s = (-s1_y * (p0.X() - p2.X()) + s1_x * (p0.Y() - p2.Y())) / (-s2_x * s1_y + s1_x * s2_y);
    double t = ( s2_x * (p0.Y() - p2.Y()) - s2_y * (p0.X() - p2.X())) / (-s2_x * s1_y + s1_x * s2_y);
    return (s > 0 && s < 1 && t > 0 && t < 1);
}

bool CheckEdgeIntersection(vector<Point> & hull, Point curEdgeStart, Point curEdgeEnd, Point checkEdgeStart, Point checkEdgeEnd)
{
    for (int i = 0; i < hull.size() - 2; i++) {
        int e1 = i;
        int e2 = i + 1;
        Point p1 = hull[e1];
        Point p2 = hull[e2];

        if (curEdgeStart == p1 && curEdgeEnd == p2) {
            continue;
        }

        if (CheckEdgeIntersection(checkEdgeStart, checkEdgeEnd, p1, p2)) {
            return true;
        }
    }
    return false;
}

/*Point NearestInnerPoint(Point edgeStart, Point edgeEnd, vector<Point> &points, vector<Point> &hull, bool * found)
{
    Point result;
    double distance = 0;
    *found = false;

    QVector<Point> &points;
    QVector<Point> &hull;
    
    foreach (Point p, points) 
    {
        // Skip points that are already in he hull
        if (hull.contains(p)) {
            continue;
        }
        /*if (!IsPointInsidePolygon(p, hull)) {
            continue;
        }*/

/*        double d = Distance(p, edgeStart, edgeEnd);
        bool skip = false;
        for (int i = 0; !skip && i < hull.size() - 1; i++) {
            double dTmp = Distance(p, hull[i], hull[i + 1]);
            skip |= dTmp < d;
        }
        if (skip) {
            continue;
        }

        if (!(*found) || distance > d) {
            result = p;
            distance = d;
            *found = true;
        }
    }
    return result;
}*/

vector<Point> FindConvexHull(vector<Point> & points)
{
    vector<Point> P = points;
    vector<Point> H;

    // Sort P by x and y
    for (int i = 0; i < P.size(); i++) {
        for (int j = i + 1; j < P.size(); j++) {
            if (P[j].X() < P[i].X() || (P[j].X() == P[i].X() && P[j].Y() < P[i].Y())) {
                Point tmp = P[i];
                P[i] = P[j];
                P[j] = tmp;
            }
        }
    }

    // the output array H[] will be used as the stack
    int i;                 // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    double xmin = P[0].X();
    for (i = 1; i < P.size(); i++)
        if (P[i].X() != xmin) break;
    minmax = i - 1;
    if (minmax == P.size() - 1) {       // degenerate case: all x-coords == xmin
        H.push_back(P[minmin]);
        if (P[minmax].Y() != P[minmin].Y()) // a  nontrivial segment
            H.push_back(P[minmax]);
        H.push_back(P[minmin]);            // add polygon endpoint
        return H;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = P.size() - 1;
    double xmax = P.back().X();
    for (i = P.size() - 2; i >= 0; i--)
        if (P[i].X() != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    H.push_back(P[minmin]);      // push  minmin point onto stack
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin]  with P[maxmin]
        if (IsLeft(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;           // ignore P[i] above or on the lower line

        while (H.size() > 1)         // there are at least 2 points on the stack
        {
            // test if  P[i] is left of the line at the stack top
            if (IsLeft(H[H.size() - 2], H.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            H.pop_back();         // pop top point off  stack
        }
        H.push_back(P[i]);        // push P[i] onto stack
    }

    // Next, compute the upper hull on the stack H above  the bottom hull
    if (maxmax != maxmin)      // if  distinct xmax points
         H.push_back(P[maxmax]);  // push maxmax point onto stack
    int bot = H.size();                  // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax]  with P[minmax]
        if (IsLeft( P[maxmax], P[minmax], P[i])  >= 0 && i > minmax)
            continue;           // ignore P[i] below or on the upper line

        while (H.size() > bot)     // at least 2 points on the upper stack
        {
            // test if  P[i] is left of the line at the stack top
            if (IsLeft(H[H.size() - 2], H.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            H.pop_back();         // pop top point off stack
        }
        H.push_back(P[i]);        // push P[i] onto stack
    }
    if (minmax != minmin)
        H.push_back(P[minmin]);  // push  joining endpoint onto stack

    return H;
}
/*
vector<Point> FindConcaveHull(const vector<Point> & points, double N)
{
    vector<Point> concaveList = FindConvexHull(points);

    QVector<Point> concaveList;

    for (int i = 0; i < concaveList.size() - 1; i++) {
        // Find the nearest inner point pk ∈ G from the edge (ci1, ci2);
        Point ci1 = concaveList[i];
        Point ci2 = concaveList[i + 1];

        bool found;
        Point pk = NearestInnerPoint(ci1, ci2, points, concaveList, &found);
        if (!found || concaveList.contains(pk)) {
            continue;
        }

        double eh = Distance(ci1, ci2);  // the lenght of the edge
        QList<Point> tmp;
        tmp.push_back(ci1);
        tmp.push_back(ci2);
        double dd = DecisionDistance(pk, tmp);

        if (eh / dd > N) 
        {
            // Check that new candidate edge will not intersect existing edges.
            bool intersects = CheckEdgeIntersection(concaveList, ci1, ci2, ci1, pk);
            intersects |= CheckEdgeIntersection(concaveList, ci1, ci2, pk, ci2);
            if (!intersects) {
                concaveList.insert(concaveList.begin() + i + 1, pk);
                i--;
            }
        }
    }
    return concaveList;
}*/
/*
// Iteratively call the main algorithm with an increasing k until success
auto ConcaveHull(PointVector &dataset, size_t k, bool iterate) -> PointVector
{
	while (k < dataset.size())
		{
		PointVector hull;
		if (ConcaveHull(dataset, k, hull) || !iterate)
			{
			return hull;
			}
		k++;
		}

	return{};
}

// The main algorithm from the Moreira-Santos paper.
auto ConcaveHull(PointVector &pointList, size_t k, PointVector &hull) -> bool
{
	hull.clear();

	if (pointList.size() < 3)
	{
		return true;
	}
	if (pointList.size() == 3)
	{
		hull = pointList;
		return true;
	}

	// construct a randomized kd-tree index using 4 kd-trees
	// 2 columns, but stride = 24 bytes in width (x, y, ignoring id)
	flann::Matrix<double> matrix(&(pointList.front().x), pointList.size(), 2, stride);
	flann::Index<flann::L2<double>> flannIndex(matrix, flann::KDTreeIndexParams(4));
	flannIndex.buildIndex();

	std::cout << "\rFinal 'k'        : " << k;

	// Initialise hull with the min-y point
	Point firstPoint = FindMinYPoint(pointList);
	AddPoint(hull, firstPoint);

	// Until the hull is of size > 3 we want to ignore the first point from nearest neighbour searches
	Point currentPoint = firstPoint;
	flannIndex.removePoint(firstPoint.id);

	double prevAngle = 0.0;
	int step = 1;

	// Iterate until we reach the start, or until there's no points left to process
	while ((!PointsEqual(currentPoint, firstPoint) || step == 1) && hull.size() != pointList.size())
		{
		if (step == 4)
			{
			// Put back the first point into the dataset and into the flann index
			firstPoint.id = pointList.size();
			flann::Matrix<double> firstPointMatrix(&firstPoint.x, 1, 2, stride);
			flannIndex.addPoints(firstPointMatrix);
			}

		PointValueVector kNearestNeighbours = NearestNeighboursFlann(flannIndex, currentPoint, k);
		PointVector cPoints = SortByAngle(kNearestNeighbours, currentPoint, prevAngle);

		bool its = true;
		size_t i = 0;

		while (its && i < cPoints.size())
			{
			size_t lastPoint = 0;
			if (PointsEqual(cPoints[i], firstPoint))
				lastPoint = 1;

			size_t j = 2;
			its = false;

			while (!its && j < hull.size() - lastPoint)
				{
				auto line1 = std::make_pair(hull[step - 1], cPoints[i]);
				auto line2 = std::make_pair(hull[step - j - 1], hull[step - j]);
				its = Intersects(line1, line2);
				j++;
				}

			if (its)
				i++;
			}

		if (its)
			return false;

		currentPoint = cPoints[i];

		AddPoint(hull, currentPoint);

		prevAngle = Angle(hull[step], hull[step - 1]);

		flannIndex.removePoint(currentPoint.id);

		step++;
		}

	// The original points less the points belonging to the hull need to be fully enclosed by the hull in order to return true.
	PointVector dataset = pointList;

	auto newEnd = RemoveHull(dataset, hull);
	bool allEnclosed = MultiplePointInPolygon(begin(dataset), newEnd, hull);

	return allEnclosed;
}
*/


/*
vector<Point> ConcaveHull(vector<Point> &pointList, size_t k)
{
        vector<Point> hull;
	hull.clear();

	if (pointList.size() <= 3)
	{
		hull = pointList;
		return hull;
	}

	// construct a randomized kd-tree index using 4 kd-trees
	// 2 columns, but stride = 24 bytes in width (x, y, ignoring id)
	flann::Matrix<double> matrix(&(pointList.front().x), pointList.size(), 2, stride);
	flann::Index<flann::L2<double>> flannIndex(matrix, flann::KDTreeIndexParams(4));
	flannIndex.buildIndex();

	std::cout << "\rFinal 'k'        : " << k;

	// Initialise hull with the min-y point
	Point firstPoint = FindMinYPoint(pointList);
	//AddPoint(hull, firstPoint);
        hull.push_back(firstPoint);

	// Until the hull is of size > 3 we want to ignore the first point from nearest neighbour searches
	Point currentPoint = firstPoint;
	flannIndex.removePoint(firstPoint.id);

	double prevAngle = 0.0;
	int step = 1;

	// Iterate until we reach the start, or until there's no points left to process
	while ((!PointsEqual(currentPoint, firstPoint) || step == 1) && hull.size() != pointList.size())
        {
		if (step == 4)
                {
                    // Put back the first point into the dataset and into the flann index
                    firstPoint.id = pointList.size();
                    flann::Matrix<double> firstPointMatrix(&firstPoint.x, 1, 2, stride);
                    flannIndex.addPoints(firstPointMatrix);
                }

		PointValueVector kNearestNeighbours = NearestNeighboursFlann(flannIndex, currentPoint, k);
		PointVector cPoints = SortByAngle(kNearestNeighbours, currentPoint, prevAngle);

		bool its = true;
		size_t i = 0;

		while (its && i < cPoints.size())
                {
			size_t lastPoint = 0;
			if (PointsEqual(cPoints[i], firstPoint))
				lastPoint = 1;

			size_t j = 2;
			its = false;

			while (!its && j < hull.size() - lastPoint)
                        {
				auto line1 = std::make_pair(hull[step - 1], cPoints[i]);
				auto line2 = std::make_pair(hull[step - j - 1], hull[step - j]);
				its = Intersects(line1, line2);
				j++;
                        }

			if (its)
				i++;
                }

		if (its)
			return false;

		currentPoint = cPoints[i];

		//AddPoint(hull, currentPoint);
                hull.push_back(currentPoint);

		prevAngle = Angle(hull[step], hull[step - 1]);

		flannIndex.removePoint(currentPoint.id);

		step++;
        }

	// The original points less the points belonging to the hull need to be fully enclosed by the hull in order to return true.
	PointVector dataset = pointList;

	auto newEnd = RemoveHull(dataset, hull);
	bool allEnclosed = MultiplePointInPolygon(begin(dataset), newEnd, hull);

	return allEnclosed;
}

// Compare a and b for equality
auto Equal(double a, double b) -> bool
{
	return fabs(a - b) <= DBL_EPSILON;
}

// Compare whether two points have the same x and y
auto PointsEqual(Point &a, Point &b) -> bool
{
	return Equal(a.X(), b.X()) && Equal(a.Y(), b.Y());
}
*/


