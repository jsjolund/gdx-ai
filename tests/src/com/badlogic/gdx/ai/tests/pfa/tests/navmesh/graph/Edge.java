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

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.utils.StringBuilder;

/** An {@link Edge} is a one way connection between two {@link Triangle} in the {@link NavMeshGraph}.
 *
 * @author jsjolund */
public class Edge implements Connection<Triangle> {

	/** Right vertex from the point of view of {@link Edge#toNode}. */
	public final Vertex rightVertex;
	/** Left vertex from the point of view of {@link Edge#toNode}. */
	public final Vertex leftVertex;
	/** Left and right vertices are calculated from the point of view of this triangle. */
	public final Triangle fromNode;
	/** Triangle on the other side of the edge to {@link Edge#fromNode}. */
	public final Triangle toNode;

	public Edge (Triangle fromNode, Triangle toNode, Vertex rightVertex, Vertex leftVertex) {
		this.fromNode = fromNode;
		this.toNode = toNode;
		this.rightVertex = rightVertex;
		this.leftVertex = leftVertex;
	}

	@Override
	public float getCost () {
		return 1;
	}

	@Override
	public Triangle getFromNode () {
		return fromNode;
	}

	@Override
	public Triangle getToNode () {
		return toNode;
	}

	@Override
	public String toString () {
		final StringBuilder sb = new StringBuilder("Edge{");
		sb.append("fromNode=").append(fromNode.graphIndex);
		sb.append(", toNode=").append(toNode == null ? "null" : toNode.graphIndex);
		sb.append(", rightVertex=").append(rightVertex);
		sb.append(", leftVertex=").append(leftVertex);
		sb.append('}');
		return sb.toString();
	}
}
