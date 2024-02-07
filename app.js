var b2Vec2 = Box2D.Common.Math.b2Vec2,
    b2AABB = Box2D.Collision.b2AABB,
    b2BodyDef = Box2D.Dynamics.b2BodyDef,
    b2Body = Box2D.Dynamics.b2Body,
    b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
    b2Fixture = Box2D.Dynamics.b2Fixture,
    b2World = Box2D.Dynamics.b2World,
    b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
    b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
    b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
    b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef;

var SCALE = 30, canvas, context, world, mouseJoint, mouseX, mouseY, canvasPosition;

function init() {
    world = new b2World(new b2Vec2(0, 10), true);
    canvas = document.getElementById('physicsCanvas');
    canvas.width = 640;
    canvas.height = 480;
    context = canvas.getContext('2d');
    canvasPosition = getElementPosition(canvas);

    setupDebugDraw();
    createGardeningBox(320, 450, 500, 30);
    fillWithDirt(320, 300, 25, 40, 10);

    setupMouseHandling();

    animate();
}

function setupDebugDraw() {
    var debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(context);
    debugDraw.SetDrawScale(SCALE);
    debugDraw.SetFillAlpha(0.3);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    world.SetDebugDraw(debugDraw);
}

function createGardeningBox(x, y, width, height) {
    var bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_staticBody;
    bodyDef.position.Set(x / SCALE, y / SCALE);
    var fixtureDef = new b2FixtureDef;
    fixtureDef.density = 1.0;
    fixtureDef.friction = 0.5;
    fixtureDef.restitution = 0.2;
    fixtureDef.shape = new b2PolygonShape;
    fixtureDef.shape.SetAsBox(width / 2 / SCALE, height / 2 / SCALE);
    var body = world.CreateBody(bodyDef);
    body.CreateFixture(fixtureDef);
}

function fillWithDirt(centerX, centerY, radius, count, spacing) {
    for(let i = 0; i < count; i++) {
        for(let j = 0; j < count; j++) {
            createCircle(centerX + spacing * (i - count / 2), centerY + spacing * (j - count / 2), radius);
        }
    }
}

function createCircle(x, y, radius) {
    var bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_dynamicBody;
    bodyDef.position.Set(x / SCALE, y / SCALE);
    var fixtureDef = new b2FixtureDef;
    fixtureDef.density = 1.0;
    fixtureDef.friction = 0.5;
    fixtureDef.restitution = 0.3;
    fixtureDef.shape = new b2CircleShape(radius / SCALE);
    var body = world.CreateBody(bodyDef);
    body.CreateFixture(fixtureDef);
}

function animate() {
    requestAnimationFrame(animate);
    world.Step(1 / 60, 10, 10);
    world.DrawDebugData();
    world.ClearForces();
}

function getElementPosition(element) {
    var elem = element, tagname = "", x = 0, y = 0;
    while ((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
       y += elem.offsetTop;
       x += elem.offsetLeft;
       tagname = elem.tagName.toUpperCase();
       if(tagname == "BODY") elem = 0;
       if(typeof(elem) == "object") {
          if(typeof(elem.offsetParent) == "object") elem = elem.offsetParent;
       }
    }
    return {x: x, y: y};
}

function setupMouseHandling() {
    var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
    var canvasPosition = getElementPosition(canvas);

    document.addEventListener("mousedown", function(e) {
        isMouseDown = true;
        handleMouseMove(e);
        document.addEventListener("mousemove", handleMouseMove, true);
    }, true);

    document.addEventListener("mouseup", function() {
        document.removeEventListener("mousemove", handleMouseMove, true);
        isMouseDown = false;
        mouseX = undefined;
        mouseY = undefined;
        if (mouseJoint) {
            world.DestroyJoint(mouseJoint);
            mouseJoint = null;
        }
    }, true);

    function handleMouseMove(e) {
        mouseX = (e.clientX - canvasPosition.x) / SCALE;
        mouseY = (e.clientY - canvasPosition.y) / SCALE;
        if (isMouseDown && !mouseJoint) {
            var body = getBodyAtMouse();
            if (body) {
                var md = new b2MouseJointDef();
                md.bodyA = world.GetGroundBody();
                md.bodyB = body;
                md.target.Set(mouseX, mouseY);
                md.collideConnected = true;
                md.maxForce = 300.0 * body.GetMass();
                mouseJoint = world.CreateJoint(md);
                body.SetAwake(true);
            }
        }
        
        if (mouseJoint) {
            mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
        }
    }

    function getBodyAtMouse() {
        mousePVec = new b2Vec2(mouseX, mouseY);
        var aabb = new b2AABB();
        aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
        aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);

        // Query the world for overlapping shapes.
        selectedBody = null;
        world.QueryAABB(getBodyCB, aabb);
        return selectedBody;
    }

    function getBodyCB(fixture) {
        if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
           if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
              selectedBody = fixture.GetBody();
              return false;
           }
        }
        return true;
    }
}

init();
