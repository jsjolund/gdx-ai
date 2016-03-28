/*******************************************************************************
 * Copyright 2015 See AUTHORS file.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.GeometryUtils;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.NavMesh;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Edge;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Vertex;
import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Plane;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.Ray;
import com.badlogic.gdx.utils.Array;

import java.util.Iterator;

/** Calculates the shortest line path through a path of triangles from the navigation mesh. Each triangle in the path must be
 * connected to another triangle with an {@link Edge}.
 *
 * @author jsjolund */
public class NavMeshPointPath implements Iterable<Vector3> {

	/** A point where an edge on the navmesh is crossed. */
	class EdgePoint {
		/** Triangle which must be crossed to reach the next path point. */
		public Triangle toNode;
		/** Triangle which was crossed to reach this point. */
		public Triangle fromNode;
		/** Path edges connected to this point. Can be used for spline generation at some point perhaps... */
		public final Array<Edge> connectingEdges = new Array<Edge>();
		/** The point where the path crosses an edge. */
		public final Vector3 point;

		@Override
		public String toString () {
			final StringBuilder sb = new StringBuilder("EdgePoint{");
			sb.append("toNode=").append(toNode);
			sb.append(", fromNode=").append(fromNode);
			sb.append(", connectingEdges=").append(connectingEdges);
			sb.append(", point=").append(point);
			sb.append('}');
			return sb.toString();
		}

		public EdgePoint (Vector3 point, Triangle toNode) {
			this.point = point;
			this.toNode = toNode;
		}

	}

	/** Plane funnel for the Simple Stupid Funnel Algorithm */
	class Funnel {

		public final Plane leftPlane = new Plane();
		public final Plane rightPlane = new Plane();
		public final Vector3 leftPortal = new Vector3();
		public final Vector3 rightPortal = new Vector3();
		public final Vector3 pivot = new Vector3();

		public void setLeftPlane (Vector3 pivot, Vector3 leftEdgeVertex) {
			leftPlane.set(pivot, tmp1.set(pivot).add(NavMesh.UP), leftEdgeVertex);
			leftPortal.set(leftEdgeVertex);
		}

		public void setRightPlane (Vector3 pivot, Vector3 rightEdgeVertex) {
			rightPlane.set(pivot, tmp1.set(pivot).add(NavMesh.UP), rightEdgeVertex);
			rightPlane.normal.scl(-1);
			rightPlane.d = -rightPlane.d;
			rightPortal.set(rightEdgeVertex);
		}

		public void setPlanes (Vector3 pivot, Edge edge) {
			setLeftPlane(pivot, edge.leftVertex);
			setRightPlane(pivot, edge.rightVertex);
		}

		public Plane.PlaneSide sideLeftPlane (Vector3 point) {
			return leftPlane.testPoint(point);
		}

		public Plane.PlaneSide sideRightPlane (Vector3 point) {
			return rightPlane.testPoint(point);
		}
	}

	private final Plane crossingPlane = new Plane();
	private final Vector3 tmp1 = new Vector3();
	private final Vector3 tmp2 = new Vector3();
	private Array<Connection<Triangle>> nodes;
	private Vector3 start;
	private Vector3 end;
	private Triangle startTri;
	private EdgePoint lastPointAdded;
	private final Array<Vector3> vectors = new Array<Vector3>();
	private final Array<EdgePoint> pathPoints = new Array<EdgePoint>();
	private Edge lastEdge;

	@Override
	public Iterator<Vector3> iterator () {
		return vectors.iterator();
	}

	private Edge getEdge (int index) {
		return (Edge)((index == nodes.size) ? lastEdge : nodes.get(index));
	}

	private int numEdges () {
		return nodes.size + 1;
	}

	/** Calculate the shortest path through the navigation mesh triangles.
	 *
	 * @param trianglePath */
	public void calculateForGraphPath (NavMeshGraphPath trianglePath) {
		clear();
		nodes = trianglePath.nodes;
		this.start = new Vector3(trianglePath.start);
		this.end = new Vector3(trianglePath.end);
		this.startTri = trianglePath.startTri;

		// Check that the start point is actually inside the start triangle, if not, project it to the closest
		// triangle edge. Otherwise the funnel calculation might generate spurious path segments.
		Ray ray = new Ray(tmp1.set(NavMesh.UP).scl(1000).add(start), tmp2.set(NavMesh.DOWN));
		if (!Intersector.intersectRayTriangle(ray, startTri.a, startTri.b, startTri.c, null)) {
			float minDst = Float.POSITIVE_INFINITY;
			Vector3 projection = new Vector3();
			Vector3 newStart = new Vector3();
			float dst;
			// A-B
			if ((dst = GeometryUtils.nearestSegmentPointSquareDistance(projection, startTri.a, startTri.b, start)) < minDst) {
				minDst = dst;
				newStart.set(projection);
			}
			// B-C
			if ((dst = GeometryUtils.nearestSegmentPointSquareDistance(projection, startTri.b, startTri.c, start)) < minDst) {
				minDst = dst;
				newStart.set(projection);
			}
			// C-A
			if ((dst = GeometryUtils.nearestSegmentPointSquareDistance(projection, startTri.c, startTri.a, start)) < minDst) {
				minDst = dst;
				newStart.set(projection);
			}
			start.set(newStart);
		}
		if (nodes.size == 0) {
			addPoint(start, startTri).fromNode = startTri;
			addPoint(end, startTri).fromNode = startTri;
		} else {
			lastEdge = new Edge(nodes.get(nodes.size - 1).getToNode(), nodes.get(nodes.size - 1).getToNode(),
				new Vertex((short)-1, end), new Vertex((short)-1, end));
			calculateEdgePoints();
		}
	}

	/** Clear the stored path data. */
	public void clear () {
		vectors.clear();
		pathPoints.clear();
		start = null;
		end = null;
		startTri = null;
		lastPointAdded = null;
		lastEdge = null;
	}

	/** A path point which crosses one or more edges in the navigation mesh.
	 *
	 * @param index
	 * @return */
	public Vector3 getVector (int index) {
		return vectors.get(index);
	}

	/** The number of path points.
	 *
	 * @return */
	public int getSize () {
		return vectors.size;
	}

	/** All vectors in the path.
	 *
	 * @return */
	public Array<Vector3> getVectors () {
		return vectors;
	}

	/** The triangle which must be crossed to reach the next path point.
	 *
	 * @param index
	 * @return */
	public Triangle getToTriangle (int index) {
		return pathPoints.get(index).toNode;
	}

	/** The triangle from which must be crossed to reach this point.
	 *
	 * @param index
	 * @return */
	public Triangle getFromTriangle (int index) {
		return pathPoints.get(index).fromNode;
	}

	/** The navmesh edge(s) crossed at this path point.
	 *
	 * @param index
	 * @return */
	public Array<Edge> getCrossedEdges (int index) {
		return pathPoints.get(index).connectingEdges;
	}

	private EdgePoint addPoint (Vector3 point, Triangle toNode) {
		return addPoint(new EdgePoint(point, toNode));
	}

	private EdgePoint addPoint (EdgePoint edgePoint) {
		vectors.add(edgePoint.point);
		pathPoints.add(edgePoint);
		lastPointAdded = edgePoint;
		return edgePoint;
	}

	/** Calculate the shortest point path through the path triangles, using the Simple Stupid Funnel Algorithm.
	 *
	 * @return */
	private void calculateEdgePoints () {
		Edge edge = getEdge(0);
		addPoint(start, edge.fromNode);
		lastPointAdded.fromNode = edge.fromNode;

		Funnel funnel = new Funnel();
		funnel.pivot.set(start);
		funnel.setPlanes(funnel.pivot, edge);

		int leftIndex = 0;
		int rightIndex = 0;
		int lastRestart = 0;

		for (int i = 1; i < numEdges(); ++i) {
			edge = getEdge(i);

			Plane.PlaneSide leftPlaneLeftDP = funnel.sideLeftPlane(edge.leftVertex);
			Plane.PlaneSide leftPlaneRightDP = funnel.sideLeftPlane(edge.rightVertex);
			Plane.PlaneSide rightPlaneLeftDP = funnel.sideRightPlane(edge.leftVertex);
			Plane.PlaneSide rightPlaneRightDP = funnel.sideRightPlane(edge.rightVertex);

			if (rightPlaneRightDP != Plane.PlaneSide.Front) {
				if (leftPlaneRightDP != Plane.PlaneSide.Front) {
					// Tighten the funnel.
					funnel.setRightPlane(funnel.pivot, edge.rightVertex);
					rightIndex = i;
				} else {
					// Right over left, insert left to path and restart scan from portal left point.
					calculateEdgeCrossings(lastRestart, leftIndex, funnel.pivot, funnel.leftPortal);
					funnel.pivot.set(funnel.leftPortal);
					i = leftIndex;
					rightIndex = i;
					if (i < numEdges() - 1) {
						lastRestart = i;
						funnel.setPlanes(funnel.pivot, getEdge(i + 1));
						continue;
					}
					break;
				}
			}
			if (leftPlaneLeftDP != Plane.PlaneSide.Front) {
				if (rightPlaneLeftDP != Plane.PlaneSide.Front) {
					// Tighten the funnel.
					funnel.setLeftPlane(funnel.pivot, edge.leftVertex);
					leftIndex = i;
				} else {
					// Left over right, insert right to path and restart scan from portal right point.
					calculateEdgeCrossings(lastRestart, rightIndex, funnel.pivot, funnel.rightPortal);
					funnel.pivot.set(funnel.rightPortal);
					i = rightIndex;
					leftIndex = i;
					if (i < numEdges() - 1) {
						lastRestart = i;
						funnel.setPlanes(funnel.pivot, getEdge(i + 1));
						continue;
					}
					break;
				}
			}
		}
		calculateEdgeCrossings(lastRestart, numEdges() - 1, funnel.pivot, end);

		for (int i = 1; i < pathPoints.size; i++) {
			EdgePoint p = pathPoints.get(i);
			p.fromNode = pathPoints.get(i - 1).toNode;
		}
	}

	/** Store all edge crossing points between the start and end indices. If the path crosses exactly the start or end points
	 * (which is quite likely), store the edges in order of crossing in the EdgePoint data structure.
	 * <p/>
	 * Edge crossings are calculated as intersections with the plane from the start, end and up vectors.
	 *
	 * @param startIndex
	 * @param endIndex
	 * @param startPoint
	 * @param endPoint */
	private void calculateEdgeCrossings (int startIndex, int endIndex, Vector3 startPoint, Vector3 endPoint) {

		if (startIndex >= numEdges() || endIndex >= numEdges()) {
			return;
		}
		crossingPlane.set(startPoint, tmp1.set(startPoint).add(NavMesh.UP), endPoint);

		EdgePoint previousLast = lastPointAdded;

		Edge edge = getEdge(endIndex);
		EdgePoint end = new EdgePoint(new Vector3(endPoint), edge.toNode);

		for (int i = startIndex; i < endIndex; i++) {
			edge = getEdge(i);

			if (edge.rightVertex.equals(startPoint) || edge.leftVertex.equals(startPoint)) {
				previousLast.toNode = edge.toNode;
				if (!previousLast.connectingEdges.contains(edge, true)) {
					previousLast.connectingEdges.add(edge);
				}

			} else if (edge.leftVertex.equals(endPoint) || edge.rightVertex.equals(endPoint)) {
				if (!end.connectingEdges.contains(edge, true)) {
					end.connectingEdges.add(edge);
				}

			} else if (Intersector.intersectSegmentPlane(edge.leftVertex, edge.rightVertex, crossingPlane, tmp1)
				&& !Float.isNaN(tmp1.x + tmp1.y + tmp1.z)) {
				if (i != startIndex || i == 0) {
					lastPointAdded.toNode = edge.fromNode;
					EdgePoint crossing = new EdgePoint(new Vector3(tmp1), edge.toNode);
					crossing.connectingEdges.add(edge);
					addPoint(crossing);
				}
			}
		}
		if (endIndex < numEdges() - 1) {
			end.connectingEdges.add(getEdge(endIndex));
		}
		if (!lastPointAdded.equals(end)) {
			addPoint(end);
		}
	}

}
