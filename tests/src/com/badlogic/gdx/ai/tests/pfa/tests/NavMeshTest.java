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

package com.badlogic.gdx.ai.tests.pfa.tests;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ai.tests.PathFinderTests;
import com.badlogic.gdx.ai.tests.pfa.PathFinderTestBase;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.MyShapeRenderer;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.NavMesh;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.NavMeshDebugDrawer;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.heuristics.ClosestCentroidsHeuristic;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.heuristics.ClosestTriangleMidpointHeuristic;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.heuristics.ClosestVerticesHeuristic;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.heuristics.NavMeshHeuristic;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path.NavMeshGraphPath;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path.NavMeshPointPath;
import com.badlogic.gdx.assets.AssetManager;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.Ray;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.scenes.scene2d.InputEvent;
import com.badlogic.gdx.scenes.scene2d.ui.ButtonGroup;
import com.badlogic.gdx.scenes.scene2d.ui.CheckBox;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.utils.ClickListener;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Disposable;

/** @author jsjolund */
public class NavMeshTest extends PathFinderTestBase {

	final static String TAG = "NavMeshTest";
	static boolean shadows = false;
	static boolean initialized = false;
	PerspectiveCamera camera;
	CameraInputController cameraController;
	BitmapFont font;
	Environment environment;
	DirectionalLight light;
	ModelBatch shadowBatch;
	ModelBatch modelBatch;
	SpriteBatch spriteBatch;
	Array<Disposable> disposables;
	AssetManager assets;
	MyShapeRenderer shapeRenderer;
	NavMeshEnum currentNavMesh;
	NavMeshHeuristicEnum currentHeuristic;

	public NavMeshTest (PathFinderTests container) {
		super(container, "Navigation Mesh Test");
	}

	protected void pathFind () {
		if (currentNavMesh.toTri != null && currentNavMesh.fromTri != null
			&& !Float.isNaN(currentNavMesh.toPoint.x + currentNavMesh.toPoint.y + currentNavMesh.toPoint.z)
			&& !Float.isNaN(currentNavMesh.fromPoint.x + currentNavMesh.fromPoint.y + currentNavMesh.fromPoint.z)) {

			Gdx.app.debug(TAG, "Finding path between " + currentNavMesh.fromTri + " at " + currentNavMesh.fromPoint + " to "
				+ currentNavMesh.toTri + " at " + currentNavMesh.toPoint);

			currentHeuristic.heuristic.searchedNodes.clear();
			currentNavMesh.pointPath.clear();
			currentNavMesh.graphPath.clear();

			currentNavMesh.mesh.getPath(currentNavMesh.fromTri, currentNavMesh.fromPoint, currentNavMesh.toTri,
				currentNavMesh.toPoint, currentNavMesh.graphPath);
			currentNavMesh.pointPath.calculateForGraphPath(currentNavMesh.graphPath);
		}
	}

	@Override
	public void create () {

		Gdx.app.setLogLevel(Application.LOG_DEBUG);
		if (!initialized) {
			Bullet.init();
			initialized = true;
		}

		currentNavMesh = NavMeshEnum.MESH1;
		currentHeuristic = NavMeshHeuristicEnum.CLOSEST_MIDPOINT;
		NavMesh.UP.set(Vector3.Y);
		NavMesh.DOWN.set(Vector3.Y).scl(-1);

		disposables = new Array<Disposable>();

		assets = new AssetManager();
		for (NavMeshEnum model : NavMeshEnum.values())
			assets.load(model.path, Model.class);
		disposables.add(assets);

		shapeRenderer = new MyShapeRenderer();
		shapeRenderer.setAutoShapeType(true);
		disposables.add(shapeRenderer);
		spriteBatch = new SpriteBatch();
		disposables.add(spriteBatch);

		environment = new Environment();
		environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.3f, 0.3f, 0.3f, 1f));
		light = shadows ? new DirectionalShadowLight(1024, 1024, 20f, 20f, 1f, 300f) : new DirectionalLight();
		light.set(0.8f, 0.8f, 0.8f, -0.5f, -1f, 0.7f);
		environment.add(light);
		if (shadows) environment.shadowMap = (DirectionalShadowLight)light;
		shadowBatch = new ModelBatch(new DepthShaderProvider());
		disposables.add(shadowBatch);
		modelBatch = new ModelBatch();
		disposables.add(modelBatch);

		float width = Gdx.graphics.getWidth();
		float height = Gdx.graphics.getHeight();
		if (width > height)
			camera = new PerspectiveCamera(67f, 3f * width / height, 3f);
		else
			camera = new PerspectiveCamera(67f, 3f, 3f * height / width);
		camera.position.set(25f, 20f, 25f);
		camera.lookAt(0, 0, 0);
		camera.update();

		cameraController = new MyCameraInputController(camera);
		inputProcessor = cameraController;

		font = new BitmapFont();
		font.setColor(Color.WHITE);
		font.setUseIntegerPositions(false);
		font.getData().setScale(0.01f);

		assets.finishLoading();
		for (NavMeshEnum m : NavMeshEnum.values()) {
			Model navMeshModel = assets.get(m.path, Model.class);
			m.modelInstance = new ModelInstance(navMeshModel);
			m.mesh = new NavMesh(navMeshModel, currentHeuristic.heuristic);
			m.graphPath = new NavMeshGraphPath();
			m.pointPath = new NavMeshPointPath();
			disposables.add(m);
		}
		setupUI();
	}

	protected void setupUI () {
		Table detailTable = new Table(container.skin);

		detailTable.top().left().add(new Label("NavMesh:", container.skin)).left().row();
		ButtonGroup<CheckBox> navMeshGroup = new ButtonGroup<CheckBox>();
		for (final NavMeshEnum navMeshEnumItem : NavMeshEnum.values()) {
			final CheckBox checkBox = new CheckBox(navMeshEnumItem.path, container.skin);
			checkBox.addListener(new ClickListener() {
				@Override
				public boolean touchDown (InputEvent event, float x, float y, int pointer, int button) {
					setCurrentNavMesh(navMeshEnumItem);
					return true;
				}
			});
			detailTable.top().left().add(checkBox).left().row();
			navMeshGroup.add(checkBox);
			if (navMeshEnumItem == currentNavMesh) checkBox.setChecked(true);
		}

		addSeparator(detailTable);
		detailTable.row();

		detailTable.top().left().add(new Label("Heuristic:", container.skin)).left().row();
		ButtonGroup<CheckBox> heuristicsGroup = new ButtonGroup<CheckBox>();
		for (final NavMeshHeuristicEnum heuristicEnumItem : NavMeshHeuristicEnum.values()) {
			final CheckBox checkBox = new CheckBox(heuristicEnumItem.toString(), container.skin);
			checkBox.addListener(new ClickListener() {
				@Override
				public boolean touchDown (InputEvent event, float x, float y, int pointer, int button) {
					setCurrentHeuristic(heuristicEnumItem);
					return true;
				}
			});
			detailTable.top().left().add(checkBox).left().row();
			heuristicsGroup.add(checkBox);
			if (currentHeuristic == heuristicEnumItem) checkBox.setChecked(true);
		}

		detailWindow = createDetailWindow(detailTable);
	}

	protected void setCurrentHeuristic (NavMeshHeuristicEnum heuristicEnumItem) {
		currentHeuristic.heuristic.searchedNodes.clear();
		currentHeuristic = heuristicEnumItem;
		for (final NavMeshEnum navMeshEnumItem : NavMeshEnum.values())
			navMeshEnumItem.mesh.setHeuristic(heuristicEnumItem.heuristic);
		pathFind();
	}

	protected void setCurrentNavMesh (NavMeshEnum navMeshEnumItem) {
		currentNavMesh = navMeshEnumItem;
		currentNavMesh.clear();
		currentHeuristic.heuristic.searchedNodes.clear();
	}

	@Override
	public void render () {
		Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

		camera.update();
		cameraController.update();

		if (shadows) {
			((DirectionalShadowLight)light).begin(Vector3.Zero, camera.direction);
			shadowBatch.begin(((DirectionalShadowLight)light).getCamera());
			shadowBatch.render(currentNavMesh.modelInstance, environment);
			shadowBatch.end();
			((DirectionalShadowLight)light).end();
		}

		modelBatch.begin(camera);
		modelBatch.render(currentNavMesh.modelInstance, environment);
		modelBatch.end();

		shapeRenderer.setProjectionMatrix(camera.combined);
		shapeRenderer.begin();

		renderNavMeshDebug();
	}

	protected void renderNavMeshDebug () {
		Gdx.gl.glEnable(GL20.GL_BLEND);
		Gdx.gl.glBlendFunc(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);

		shapeRenderer.set(MyShapeRenderer.ShapeType.Line);
		NavMeshDebugDrawer.drawNavMeshTriangles(shapeRenderer, null, currentNavMesh.mesh, Color.DARK_GRAY);

		shapeRenderer.set(MyShapeRenderer.ShapeType.Filled);
		if (!Float.isNaN(currentNavMesh.fromPoint.x + currentNavMesh.fromPoint.y + currentNavMesh.fromPoint.z))
			NavMeshDebugDrawer.drawVertex(shapeRenderer, currentNavMesh.fromPoint, 0.1f, Color.WHITE);
		if (!Float.isNaN(currentNavMesh.toPoint.x + currentNavMesh.toPoint.y + currentNavMesh.toPoint.z))
			NavMeshDebugDrawer.drawVertex(shapeRenderer, currentNavMesh.toPoint, 0.1f, Color.WHITE);
		for (Triangle searchedNode : currentHeuristic.heuristic.searchedNodes) {
			NavMeshDebugDrawer.drawTriangle(shapeRenderer, searchedNode, Color.LIME, 0.3f);
		}
		NavMeshDebugDrawer.drawPathTriangles(shapeRenderer, null, currentNavMesh.graphPath, Color.CYAN, 0.5f);
		if (currentNavMesh.fromTri != null)
			NavMeshDebugDrawer.drawTriangle(shapeRenderer, currentNavMesh.fromTri, Color.GREEN, 0.5f);
		if (currentNavMesh.toTri != null) NavMeshDebugDrawer.drawTriangle(shapeRenderer, currentNavMesh.toTri, Color.RED, 0.5f);

		shapeRenderer.set(MyShapeRenderer.ShapeType.Line);
		NavMeshDebugDrawer.drawSharedEdgesInPath(shapeRenderer, null, currentNavMesh.graphPath, Color.RED, Color.GREEN, 0.3f);
		NavMeshDebugDrawer.drawPathLines(shapeRenderer, currentNavMesh.pointPath, Color.YELLOW);

		shapeRenderer.set(MyShapeRenderer.ShapeType.Filled);
		NavMeshDebugDrawer.drawPathPoints(shapeRenderer, currentNavMesh.pointPath, Color.YELLOW, 0.025f);

		shapeRenderer.end();

		spriteBatch.begin();
		NavMeshDebugDrawer.drawNavMeshIndices(spriteBatch, null, camera, font, currentNavMesh.mesh);
		NavMeshDebugDrawer.drawPathPointTriConnections(spriteBatch, camera, font, currentNavMesh.pointPath);
		spriteBatch.end();

		Gdx.gl.glDisable(GL20.GL_BLEND);
	}

	@Override
	public void dispose () {
		shapeRenderer.setProjectionMatrix(container.stage.getViewport().getCamera().combined);
		for (Disposable disposable : disposables)
			disposable.dispose();
		disposables.clear();

		for (NavMeshHeuristicEnum heuristicEnum : NavMeshHeuristicEnum.values())
			heuristicEnum.heuristic.searchedNodes.clear();

		modelBatch = null;
		shadowBatch = null;
		shapeRenderer = null;
		spriteBatch = null;
		currentNavMesh = null;

		if (shadows) ((DirectionalShadowLight)light).dispose();
		light = null;
	}

	protected enum NavMeshEnum implements Disposable {
		MESH1("data/navmesh1.obj"), MESH2("data/navmesh2.obj");

		NavMesh mesh;
		NavMeshGraphPath graphPath;
		NavMeshPointPath pointPath;
		ModelInstance modelInstance;

		final String path;

		Triangle fromTri;
		Triangle toTri;

		Vector3 fromPoint = new Vector3(Float.NaN, Float.NaN, Float.NaN);
		Vector3 toPoint = new Vector3(Float.NaN, Float.NaN, Float.NaN);

		NavMeshEnum (String path) {
			this.path = path;
		}

		public void clear () {
			graphPath.clear();
			pointPath.clear();
			fromTri = null;
			toTri = null;
			fromPoint.set(Float.NaN, Float.NaN, Float.NaN);
			toPoint.set(Float.NaN, Float.NaN, Float.NaN);
		}

		@Override
		public void dispose () {
			if (mesh != null) mesh.dispose();
			mesh = null;
			modelInstance = null;
			clear();
			graphPath = null;
			pointPath = null;
		}
	}

	protected enum NavMeshHeuristicEnum {
		CLOSEST_CENTROID(new ClosestCentroidsHeuristic()), CLOSEST_VERTICES(new ClosestVerticesHeuristic()), CLOSEST_MIDPOINT(
			new ClosestTriangleMidpointHeuristic());

		NavMeshHeuristic heuristic;

		NavMeshHeuristicEnum (NavMeshHeuristic heuristic) {
			this.heuristic = heuristic;
		}
	}

	protected class MyCameraInputController extends CameraInputController {
		Ray ray = new Ray();
		boolean numberClicksEven = false;

		public MyCameraInputController (Camera camera) {
			super(camera);
		}

		@Override
		public boolean touchUp (int screenX, int screenY, int pointer, int button) {
			if (!isPanning()) {
				ray.set(camera.getPickRay(screenX, screenY));
				Triangle tri = currentNavMesh.mesh.rayTest(ray, 100, null);
				if (tri == null) {
					return true;
				} else if ((numberClicksEven = !numberClicksEven)) {
					currentNavMesh.fromTri = tri;
					Intersector.intersectRayTriangle(ray, tri.a, tri.b, tri.c, currentNavMesh.fromPoint);

				} else {
					currentNavMesh.toTri = tri;
					Intersector.intersectRayTriangle(ray, tri.a, tri.b, tri.c, currentNavMesh.toPoint);
				}
				pathFind();
			}
			super.touchUp(screenX, screenY, pointer, button);
			return true;
		}
	}

}
