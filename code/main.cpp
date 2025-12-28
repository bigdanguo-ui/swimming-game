#include "Angel.h"
#include "TriMesh.h"
#include "Camera.h"

#define STBI_WINDOWS_UTF8
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <assert.h>
#ifdef _WIN32
#include <windows.h>
#endif


struct openGLObject
{
	// 顶点数组对象
	GLuint vao;
	// 顶点缓存对象
	GLuint vbo;

	// 着色器程序
	GLuint program;
	// 着色器文件
	std::string vshader;
	std::string fshader;
	// 着色器变量
	GLuint pLocation;
	GLuint cLocation;
	GLuint nLocation;
	GLuint tLocation;

	// 投影变换变量
	GLuint modelLocation;
	GLuint viewLocation;
	GLuint projectionLocation;

	// 阴影变量
	GLuint shadowLocation;

	// 纹理变量
	GLuint useTextureLocation;
	GLuint texScaleLocation;
	GLuint texOffsetLocation;
	GLuint textureLocation;
	GLuint textureID = 0;
	int useTexture = 0;
	glm::vec2 texScale = glm::vec2(1.0f, 1.0f);
	glm::vec2 texOffset = glm::vec2(0.0f, 0.0f);
	GLuint colorTintLocation;
	glm::vec3 colorTint = glm::vec3(1.0f, 1.0f, 1.0f);
	GLuint alphaLocation;
	float alpha = 1.0f;

	// 光照变量
	GLuint useLightingLocation;
	GLuint lightPosLocation;
	GLuint lightColorLocation;
	GLuint ambientStrengthLocation;
	GLuint specStrengthLocation;
	GLuint shininessLocation;
	GLuint eyePositionLocation;
	int useLighting = 1;

	// 阴影透明度
	GLuint shadowAlphaLocation;
	float shadowAlpha = 0.45f;
};

int WIDTH = 600;
int HEIGHT = 600;

int mainWindow;
GLFWwindow* gWindow = NULL;
const char* kTextureLogPath = "texture_log.txt";

#ifdef _WIN32
std::wstring utf8ToWide(const std::string& utf8)
{
	if (utf8.empty()) {
		return std::wstring();
	}
	int size = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, NULL, 0);
	if (size <= 0) {
		return std::wstring();
	}
	std::wstring wide(static_cast<size_t>(size - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, &wide[0], size);
	return wide;
}

std::string wideToUtf8(const std::wstring& wide)
{
	if (wide.empty()) {
		return std::string();
	}
	int size = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, NULL, 0, NULL, NULL);
	if (size <= 0) {
		return std::string();
	}
	std::string utf8(static_cast<size_t>(size - 1), '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, &utf8[0], size, NULL, NULL);
	return utf8;
}

bool fileExistsWide(const std::wstring& path)
{
	DWORD attrib = GetFileAttributesW(path.c_str());
	return (attrib != INVALID_FILE_ATTRIBUTES) && ((attrib & FILE_ATTRIBUTE_DIRECTORY) == 0);
}

std::wstring getExeDir()
{
	wchar_t buffer[MAX_PATH] = { 0 };
	GetModuleFileNameW(NULL, buffer, MAX_PATH);
	std::wstring path(buffer);
	size_t pos = path.find_last_of(L"\\/");
	if (pos != std::wstring::npos) {
		path = path.substr(0, pos);
	}
	return path;
}

std::wstring normalizeSeparators(std::wstring path)
{
	for (size_t i = 0; i < path.size(); ++i) {
		if (path[i] == L'/') {
			path[i] = L'\\';
		}
	}
	return path;
}
#endif

class MatrixStack {
	int		_index;
    int		_size;
    glm::mat4*	_matrices;

public:
	MatrixStack(int numMatrices = 100):_index(0), _size(numMatrices)
        { _matrices = new glm::mat4[numMatrices]; }

    ~MatrixStack()
		{ delete[]_matrices; }

    void push(const glm::mat4& m){
		assert( _index + 1 < _size );
		_matrices[_index++] = m;	 
    }

	glm::mat4& pop(){
        assert(_index - 1 >= 0);
        _index--;
        return _matrices[_index];
    }
};


#define White	glm::vec3(1.0, 1.0, 1.0)
#define Yellow	glm::vec3(1.0, 1.0, 0.0)
#define Green	glm::vec3(0.0, 1.0, 0.0)
#define Cyan	glm::vec3(0.0, 1.0, 1.0)
#define Magenta	glm::vec3(1.0, 0.0, 1.0)
#define Red		glm::vec3(1.0, 0.0, 0.0)
#define Black	glm::vec3(0.0, 0.0, 0.0)
#define Blue	glm::vec3(0.0, 0.0, 1.0)
#define Brown	glm::vec3(0.5, 0.5, 0.5)
#define SkyBlue glm::vec3(0.25, 0.6, 0.9)

struct Robot
{
	// 关节大小
	float TORSO_HEIGHT = 4.0;
	float TORSO_WIDTH = 2.5;
	float UPPER_ARM_HEIGHT = 2.5;
	float LOWER_ARM_HEIGHT = 1.8;
	float UPPER_ARM_WIDTH =  0.8;
	float LOWER_ARM_WIDTH =  0.5;
	float UPPER_LEG_HEIGHT = 2.8;
	float LOWER_LEG_HEIGHT = 2.2;
	float UPPER_LEG_WIDTH =  1.0;
	float LOWER_LEG_WIDTH =  0.5;
	float HEAD_HEIGHT = 1.8;
	float HEAD_WIDTH = 1.5;

	// 关节角和菜单选项值
	enum {
		Torso,			// 躯干
		Head,			// 头部
		RightUpperArm,	// 右大臂
		RightLowerArm,	// 右小臂
		LeftUpperArm,	// 左大臂
		LeftLowerArm,	// 左小臂
		RightUpperLeg,	// 右大腿
		RightLowerLeg,	// 右小腿
		LeftUpperLeg,	// 左大腿
		LeftLowerLeg,	// 左小腿
	};

	// 关节角大小
	GLfloat theta[10] = {
		0.0,    // Torso
		0.0,    // Head
		0.0,    // RightUpperArm
		0.0,    // RightLowerArm
		0.0,    // LeftUpperArm
		0.0,    // LeftLowerArm
		0.0,    // RightUpperLeg
		0.0,    // RightLowerLeg
		0.0,    // LeftUpperLeg
		0.0     // LeftLowerLeg
	};
};
Robot robot;
// 被选中的物体
int Selected_mesh = robot.Torso;

struct PoolScene
{
	float GROUND_SIZE = 60.0;
	float GROUND_THICKNESS = 0.2;
	float GROUND_DROP = 0.4;
	float POOL_LENGTH = 300.0;
	float POOL_WIDTH = 150.0;
	float POOL_DEPTH = 20.0;
	float WALL_THICKNESS = 0.4;
	float DECK_BORDER = 15.0;
	float DECK_THICKNESS = 2;
	float WATER_THICKNESS = 20;
	float SKYBOX_SIZE = 1.0;

	float LADDER_HEIGHT = 2.6;
	float LADDER_WIDTH = 1.2;
	float LADDER_RAIL_THICKNESS = 0.15;
	float LADDER_RUNG_THICKNESS = 0.12;
	float LADDER_RUNG_DEPTH = 0.5;
	float LADDER_RUNG_START = 0.4;
	float LADDER_RUNG_SPACING = 0.55;
	int LADDER_RUNG_COUNT = 4;

	glm::vec3 position = glm::vec3(5.0, 0.4, -6.0);
};
PoolScene poolScene;

const bool kEnableTextures = true;
const glm::vec3 kLightPosition = glm::vec3(0.0f, 110.0f, 10.0f);
const glm::vec3 kLightColor = glm::vec3(1.0f, 1.0f, 1.0f);
const float kAmbientStrength = 0.5f;
const float kSpecStrength = 0.4f;
const float kShininess = 32.0f;
const float kCampusScale = 10.0f;
const float kCampusWallHeight = 50.0f;
const float kCampusWallThickness = 2.0f;
const float kRobotCollisionRadius = 1.2f;
const float kGravity = 25.0f;
const int kLaneCount = 5;
const int kRobotLaneIndex = 2;
const int kSecondPlayerLaneIndex = 1;
const float kRobotFacingYaw = -90.0f;
const float kPlayerYawStep = 30.0f;
const float kRaceStartInset = 6.0f;
const float kAiMinSpeed = 8.0f;
const float kAiMaxSpeed = 12.0f;
const float kPlayerStrokeBoost = 2.5f;
const float kPlayerSpeedDecay = 6.0f;
const float kPlayerMaxSpeed = 16.0f;

GLuint gSkyboxFrontTexture = 0;
GLuint gSkyboxBackTexture = 0;
GLuint gSkyboxLeftTexture = 0;
GLuint gSkyboxRightTexture = 0;
GLuint gSkyboxTopTexture = 0;
GLuint gSkyboxBottomTexture = 0;
glm::vec3 gRobotPosition = glm::vec3(0.0f);
float gRobotMoveSpeed = 10.0f;
float gRobotVelocityY = 0.0f;
float gPlayerSwimSpeed = 0.0f;
float gPlayerYaw = kRobotFacingYaw;
float gPlayerYawAccum = 0.0f;
float gPlayerScale = 1.0f;
glm::vec3 gSecondRobotPosition = glm::vec3(0.0f);
float gSecondSwimSpeed = 0.0f;
bool gSecondFinished = false;
float gSecondYaw = kRobotFacingYaw;
float gSecondYawAccum = 0.0f;
float gSecondScale = 1.0f;
float gMouseSensitivity = 0.1f;
float gHeadPitch = 0.0f;
float gCameraYawOffset = 0.0f;
float gCameraPitchOffset = 0.0f;
float gCameraFollowDistance = 100.0f;
bool gIsDragging = false;
bool gFirstMouse = true;
double gLastX = 0.0;
double gLastY = 0.0;
struct SwimmerState {
	int laneIndex = 0;
	glm::vec3 position = glm::vec3(0.0f);
	float speed = 0.0f;
	glm::vec3 tint = glm::vec3(1.0f);
	bool finished = false;
};
std::vector<SwimmerState> gAiSwimmers;
bool gRaceStarted = false;
bool gRaceFinished = false;
bool gPlayerFinished = false;
int gWinnerLane = -1;
bool gStartRequested = false;


TriMesh* Torso = new TriMesh();
TriMesh* Head = new TriMesh();
TriMesh* RightUpperArm = new TriMesh();
TriMesh* RightLowerArm = new TriMesh();
TriMesh* LeftUpperArm = new TriMesh();
TriMesh* LeftLowerArm = new TriMesh();
TriMesh* RightUpperLeg = new TriMesh();
TriMesh* RightLowerLeg = new TriMesh();
TriMesh* LeftUpperLeg = new TriMesh();
TriMesh* LeftLowerLeg = new TriMesh();

TriMesh* Ground = new TriMesh();
TriMesh* Deck = new TriMesh();
TriMesh* PoolBottom = new TriMesh();
TriMesh* PoolWall = new TriMesh();
TriMesh* PoolWater = new TriMesh();
TriMesh* Ladder = new TriMesh();
TriMesh* Skybox = new TriMesh();
TriMesh* SchoolBuilding = new TriMesh();
TriMesh* SchoolRoof = new TriMesh();
TriMesh* SchoolDoor = new TriMesh();
TriMesh* SchoolWindow = new TriMesh();
TriMesh* CampusWall = new TriMesh();
TriMesh* SwimRing = new TriMesh();
TriMesh* LaneFloat = new TriMesh();
TriMesh* SpectatorStand = new TriMesh();
TriMesh* Spectator = new TriMesh();

openGLObject TorsoObject;
openGLObject HeadObject;
openGLObject RightUpperArmObject;
openGLObject RightLowerArmObject;
openGLObject LeftUpperArmObject;
openGLObject LeftLowerArmObject;
openGLObject RightUpperLegObject;
openGLObject RightLowerLegObject;
openGLObject LeftUpperLegObject;
openGLObject LeftLowerLegObject;

openGLObject GroundObject;
openGLObject DeckObject;
openGLObject PoolBottomObject;
openGLObject PoolWallObject;
openGLObject PoolWaterObject;
openGLObject LadderObject;
openGLObject SkyboxObject;
openGLObject SchoolBuildingObject;
openGLObject SchoolRoofObject;
openGLObject SchoolDoorObject;
openGLObject SchoolWindowObject;
openGLObject CampusWallObject;
openGLObject SwimRingObject;
openGLObject LaneFloatObject;
openGLObject SpectatorStandObject;
openGLObject SpectatorObject;

Camera* camera = new Camera();

// 获取生成的所有模型，用于结束程序时释放内存
std::vector<TriMesh*> meshList;

void drawShadowMesh(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object, float planeY);
void drawScaledMesh(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object, const glm::vec3& translate, const glm::vec3& scale);
float getGroundTopY();
float getCampusHalfExtent();
float hash01(unsigned int seed);

void drawMesh(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object) {

	glBindVertexArray(object.vao);

	glUseProgram(object.program);
	GLboolean blendEnabled = glIsEnabled(GL_BLEND);
	GLboolean depthMask = GL_TRUE;
	glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
	bool useBlend = object.alpha < 0.999f;
	if (useBlend) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
	}
 
    // 父节点矩阵 * 本节点局部变换矩阵
	glUniformMatrix4fv( object.modelLocation, 1, GL_FALSE, &modelMatrix[0][0]);
	glUniformMatrix4fv( object.viewLocation, 1, GL_FALSE, &camera->viewMatrix[0][0]);
	glUniformMatrix4fv( object.projectionLocation, 1, GL_FALSE, &camera->projMatrix[0][0]);
	glUniform1i( object.shadowLocation, 0);
	glUniform1f(object.shadowAlphaLocation, object.shadowAlpha);
	glUniform1i(object.useLightingLocation, object.useLighting);
	if (object.useLighting == 1) {
		glUniform3fv(object.lightPosLocation, 1, &kLightPosition[0]);
		glUniform3fv(object.lightColorLocation, 1, &kLightColor[0]);
		glUniform3fv(object.eyePositionLocation, 1, &camera->eye[0]);
		glUniform1f(object.ambientStrengthLocation, kAmbientStrength);
		glUniform1f(object.specStrengthLocation, kSpecStrength);
		glUniform1f(object.shininessLocation, kShininess);
	}
	if (static_cast<GLint>(object.colorTintLocation) != -1) {
		glUniform3fv(object.colorTintLocation, 1, &object.colorTint[0]);
	}
	if (static_cast<GLint>(object.alphaLocation) != -1) {
		glUniform1f(object.alphaLocation, object.alpha);
	}
	if (object.useTexture == 1 && object.textureID != 0) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, object.textureID);
		glUniform1i(object.textureLocation, 0);
		glUniform1i(object.useTextureLocation, 1);
		glUniform2fv(object.texScaleLocation, 1, &object.texScale[0]);
		glUniform2fv(object.texOffsetLocation, 1, &object.texOffset[0]);
	}
	else {
		glUniform1i(object.useTextureLocation, 0);
	}
	// 绘制
	glDrawArrays(GL_TRIANGLES, 0, mesh->getPoints().size());
	if (useBlend) {
		glDepthMask(depthMask);
		if (!blendEnabled) {
			glDisable(GL_BLEND);
		}
	}
}

void drawMeshWithShadow(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object, float shadowPlaneY, bool castShadow)
{
	drawMesh(modelMatrix, mesh, object);
	if (castShadow) {
		drawShadowMesh(modelMatrix, mesh, object, shadowPlaneY);
	}
}

void drawSwimRing(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	const int segments = 16;
	const float ringRadius = 1.0f;
	const float tubeRadius = 0.25f;
	const float segmentLength = tubeRadius * 2.5f;
	for (int i = 0; i < segments; ++i) {
		float angle = 360.0f * static_cast<float>(i) / static_cast<float>(segments);
		glm::mat4 segment = glm::rotate(modelMatrix, glm::radians(angle), glm::vec3(0.0f, 1.0f, 0.0f));
		segment = glm::translate(segment, glm::vec3(ringRadius, 0.0f, 0.0f));
		segment = glm::scale(segment, glm::vec3(tubeRadius, tubeRadius, segmentLength));
		drawMeshWithShadow(segment, SwimRing, SwimRingObject, shadowPlaneY, castShadow);
	}
}

// 躯体
void torso(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	// 本节点局部变换矩阵
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, 0.5 * robot.TORSO_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.TORSO_WIDTH, robot.TORSO_HEIGHT, robot.TORSO_WIDTH));

	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, Torso, TorsoObject, shadowPlaneY, castShadow);
}

// 头部
void head(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	// 本节点局部变换矩阵
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, 0.5 * robot.HEAD_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.HEAD_WIDTH, robot.HEAD_HEIGHT, robot.HEAD_WIDTH));

	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, Head, HeadObject, shadowPlaneY, castShadow);
}


// 左大臂
void left_upper_arm(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
    // 本节点局部变换矩阵
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.UPPER_ARM_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));

	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, LeftUpperArm, LeftUpperArmObject, shadowPlaneY, castShadow);
}


// @TODO: 左小臂
void left_lower_arm(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.LOWER_ARM_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, LeftLowerArm, LeftLowerArmObject, shadowPlaneY, castShadow);

}

// @TODO: 右大臂
void right_upper_arm(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.UPPER_ARM_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, RightUpperArm, RightUpperArmObject, shadowPlaneY, castShadow);

}

// @TODO: 右小臂
void right_lower_arm(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.LOWER_ARM_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, RightLowerArm, RightLowerArmObject, shadowPlaneY, castShadow);

}

void swim_ring(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	drawSwimRing(modelMatrix, shadowPlaneY, castShadow);
}

// @TODO: 左大腿
void left_upper_leg(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, LeftUpperLeg, LeftUpperLegObject, shadowPlaneY, castShadow);

}

// @TODO: 左小腿
void left_lower_leg(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.LOWER_LEG_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, LeftLowerLeg, LeftLowerLegObject, shadowPlaneY, castShadow);
}

// @TODO: 右大腿
void right_upper_leg(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, RightUpperLeg, RightUpperLegObject, shadowPlaneY, castShadow);

}

// @TODO: 右小腿
void right_lower_leg(glm::mat4 modelMatrix, float shadowPlaneY, bool castShadow)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, glm::vec3(0.0, -0.5 * robot.LOWER_LEG_HEIGHT, 0.0));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	// 乘以来自父物体的模型变换矩阵，绘制当前物体
	drawMeshWithShadow(modelMatrix * instance, RightLowerLeg, RightLowerLegObject, shadowPlaneY, castShadow);

}

void drawSpectatorRobot(glm::mat4 modelMatrix, const glm::vec3& tint, float upperArmAngle, float lowerArmAngle,
	float shadowPlaneY, bool castShadow)
{
	openGLObject torsoObj = TorsoObject;
	openGLObject headObj = HeadObject;
	openGLObject leftUpperArmObj = LeftUpperArmObject;
	openGLObject leftLowerArmObj = LeftLowerArmObject;
	openGLObject rightUpperArmObj = RightUpperArmObject;
	openGLObject rightLowerArmObj = RightLowerArmObject;
	openGLObject leftUpperLegObj = LeftUpperLegObject;
	openGLObject leftLowerLegObj = LeftLowerLegObject;
	openGLObject rightUpperLegObj = RightUpperLegObject;
	openGLObject rightLowerLegObj = RightLowerLegObject;

	torsoObj.colorTint = tint;
	headObj.colorTint = tint;
	leftUpperArmObj.colorTint = tint;
	leftLowerArmObj.colorTint = tint;
	rightUpperArmObj.colorTint = tint;
	rightLowerArmObj.colorTint = tint;
	leftUpperLegObj.colorTint = tint;
	leftLowerLegObj.colorTint = tint;
	rightUpperLegObj.colorTint = tint;
	rightLowerLegObj.colorTint = tint;

	MatrixStack mstack;

	glm::mat4 instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, 0.5f * robot.TORSO_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.TORSO_WIDTH, robot.TORSO_HEIGHT, robot.TORSO_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, Torso, torsoObj, shadowPlaneY, castShadow);

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, robot.TORSO_HEIGHT, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, 0.5f * robot.HEAD_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.HEAD_WIDTH, robot.HEAD_HEIGHT, robot.HEAD_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, Head, headObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5f * robot.TORSO_WIDTH - 0.5f * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(upperArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftUpperArm, leftUpperArmObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_ARM_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(lowerArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftLowerArm, leftLowerArmObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5f * robot.TORSO_WIDTH + 0.5f * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-upperArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightUpperArm, rightUpperArmObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_ARM_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-lowerArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightLowerArm, rightLowerArmObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5f * robot.TORSO_WIDTH + 0.5f * robot.UPPER_LEG_WIDTH, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftUpperLeg, leftUpperLegObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftLowerLeg, leftLowerLegObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5f * robot.TORSO_WIDTH - 0.5f * robot.UPPER_LEG_WIDTH, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightUpperLeg, rightUpperLegObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightLowerLeg, rightLowerLegObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();
}

void drawSwimmerRobot(glm::mat4 modelMatrix, const glm::vec3& tint, float upperArmAngle, float lowerArmAngle,
	float upperLegAngle, float lowerLegAngle, float bodyPitch, float shadowPlaneY, bool castShadow)
{
	openGLObject torsoObj = TorsoObject;
	openGLObject headObj = HeadObject;
	openGLObject leftUpperArmObj = LeftUpperArmObject;
	openGLObject leftLowerArmObj = LeftLowerArmObject;
	openGLObject rightUpperArmObj = RightUpperArmObject;
	openGLObject rightLowerArmObj = RightLowerArmObject;
	openGLObject leftUpperLegObj = LeftUpperLegObject;
	openGLObject leftLowerLegObj = LeftLowerLegObject;
	openGLObject rightUpperLegObj = RightUpperLegObject;
	openGLObject rightLowerLegObj = RightLowerLegObject;

	torsoObj.colorTint = tint;
	headObj.colorTint = tint;
	leftUpperArmObj.colorTint = tint;
	leftLowerArmObj.colorTint = tint;
	rightUpperArmObj.colorTint = tint;
	rightLowerArmObj.colorTint = tint;
	leftUpperLegObj.colorTint = tint;
	leftLowerLegObj.colorTint = tint;
	rightUpperLegObj.colorTint = tint;
	rightLowerLegObj.colorTint = tint;

	MatrixStack mstack;

	modelMatrix = glm::rotate(modelMatrix, glm::radians(bodyPitch), glm::vec3(1.0f, 0.0f, 0.0f));

	glm::mat4 instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, 0.5f * robot.TORSO_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.TORSO_WIDTH, robot.TORSO_HEIGHT, robot.TORSO_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, Torso, torsoObj, shadowPlaneY, castShadow);

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, robot.TORSO_HEIGHT, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, 0.5f * robot.HEAD_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.HEAD_WIDTH, robot.HEAD_HEIGHT, robot.HEAD_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, Head, headObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5f * robot.TORSO_WIDTH - 0.5f * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(upperArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftUpperArm, leftUpperArmObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_ARM_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(lowerArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftLowerArm, leftLowerArmObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5f * robot.TORSO_WIDTH + 0.5f * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-upperArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightUpperArm, rightUpperArmObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_ARM_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-lowerArmAngle), glm::vec3(0.0f, 0.0f, 1.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_ARM_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightLowerArm, rightLowerArmObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5f * robot.TORSO_WIDTH + 0.5f * robot.UPPER_LEG_WIDTH, 0.0f, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(upperLegAngle), glm::vec3(1.0f, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftUpperLeg, leftUpperLegObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_LEG_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(lowerLegAngle), glm::vec3(1.0f, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, LeftLowerLeg, leftLowerLegObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5f * robot.TORSO_WIDTH - 0.5f * robot.UPPER_LEG_WIDTH, 0.0f, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-upperLegAngle), glm::vec3(1.0f, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.UPPER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightUpperLeg, rightUpperLegObj, shadowPlaneY, castShadow);

	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.UPPER_LEG_HEIGHT, 0.0f));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-lowerLegAngle), glm::vec3(1.0f, 0.0f, 0.0f));
	instance = glm::mat4(1.0f);
	instance = glm::translate(instance, glm::vec3(0.0f, -0.5f * robot.LOWER_LEG_HEIGHT, 0.0f));
	instance = glm::scale(instance, glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
	drawMeshWithShadow(modelMatrix * instance, RightLowerLeg, rightLowerLegObj, shadowPlaneY, castShadow);
	modelMatrix = mstack.pop();
}

glm::mat4 makeBillboardMatrix(const glm::vec3& position)
{
	glm::vec3 forward = glm::vec3(camera->at) - glm::vec3(camera->eye);
	if (glm::length(forward) < 0.001f) {
		forward = glm::vec3(0.0f, 0.0f, -1.0f);
	}
	forward = glm::normalize(forward);
	glm::vec3 zAxis = -forward;
	glm::vec3 up = glm::normalize(glm::vec3(camera->up));
	if (std::abs(glm::dot(up, zAxis)) > 0.99f) {
		up = glm::vec3(0.0f, 0.0f, 1.0f);
	}
	glm::vec3 right = glm::normalize(glm::cross(up, zAxis));
	glm::vec3 billboardUp = glm::normalize(glm::cross(zAxis, right));
	glm::mat4 rotation(1.0f);
	rotation[0] = glm::vec4(right, 0.0f);
	rotation[1] = glm::vec4(billboardUp, 0.0f);
	rotation[2] = glm::vec4(zAxis, 0.0f);
	return glm::translate(glm::mat4(1.0f), position) * rotation;
}

void drawPlayerNumber(const glm::mat4& modelMatrix, const glm::vec3& position, int number, const glm::vec3& tint)
{
	static const bool kSegments[10][7] = {
		{ true,  true,  true,  false, true,  true,  true  }, // 0
		{ false, false, true,  false, false, true,  false }, // 1
		{ true,  false, true,  true,  true,  false, true  }, // 2
		{ true,  false, true,  true,  false, true,  true  }, // 3
		{ false, true,  true,  true,  false, true,  false }, // 4
		{ true,  true,  false, true,  false, true,  true  }, // 5
		{ true,  true,  false, true,  true,  true,  true  }, // 6
		{ true,  false, true,  false, false, true,  false }, // 7
		{ true,  true,  true,  true,  true,  true,  true  }, // 8
		{ true,  true,  true,  true,  false, true,  true  }  // 9
	};
	if (number < 0 || number > 9) {
		return;
	}

	openGLObject digitObject = TorsoObject;
	digitObject.useTexture = 0;
	digitObject.colorTint = tint;

	float width = 0.8f;
	float height = 1.2f;
	float thickness = 0.12f;
	float depth = 0.1f;
	float halfW = width * 0.5f;
	float halfH = height * 0.5f;
	float vertLen = height * 0.5f - thickness * 0.5f;

	glm::vec3 worldPos = glm::vec3(modelMatrix * glm::vec4(position, 1.0f));
	glm::mat4 base = makeBillboardMatrix(worldPos);
	auto drawSegment = [&](const glm::vec3& offset, const glm::vec3& scale) {
		drawScaledMesh(base, Torso, digitObject, offset, scale);
	};

	if (kSegments[number][0]) {
		drawSegment(glm::vec3(0.0f, halfH, 0.0f), glm::vec3(width, thickness, depth));
	}
	if (kSegments[number][1]) {
		drawSegment(glm::vec3(-halfW, halfH * 0.5f, 0.0f), glm::vec3(thickness, vertLen, depth));
	}
	if (kSegments[number][2]) {
		drawSegment(glm::vec3(halfW, halfH * 0.5f, 0.0f), glm::vec3(thickness, vertLen, depth));
	}
	if (kSegments[number][3]) {
		drawSegment(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(width, thickness, depth));
	}
	if (kSegments[number][4]) {
		drawSegment(glm::vec3(-halfW, -halfH * 0.5f, 0.0f), glm::vec3(thickness, vertLen, depth));
	}
	if (kSegments[number][5]) {
		drawSegment(glm::vec3(halfW, -halfH * 0.5f, 0.0f), glm::vec3(thickness, vertLen, depth));
	}
	if (kSegments[number][6]) {
		drawSegment(glm::vec3(0.0f, -halfH, 0.0f), glm::vec3(width, thickness, depth));
	}
}

void drawScaledMesh(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object, const glm::vec3& translate, const glm::vec3& scale)
{
	glm::mat4 instance = glm::mat4(1.0);
	instance = glm::translate(instance, translate);
	instance = glm::scale(instance, scale);
	drawMesh(modelMatrix * instance, mesh, object);
}

glm::mat4 shadowMatrixYPlane(float planeY, const glm::vec3& lightPos)
{
	glm::vec3 lp = lightPos - glm::vec3(0.0f, planeY, 0.0f);
	float lx = lp.x;
	float ly = lp.y;
	float lz = lp.z;

	glm::mat4 shadow(
		-ly, 0.0f, 0.0f, 0.0f,
		lx, 0.0f, lz, 1.0f,
		0.0f, 0.0f, -ly, 0.0f,
		0.0f, 0.0f, 0.0f, -ly
	);

	glm::mat4 translateTo = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -planeY, 0.0f));
	glm::mat4 translateBack = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, planeY, 0.0f));
	return translateBack * shadow * translateTo;
}

void drawShadowMesh(glm::mat4 modelMatrix, TriMesh* mesh, openGLObject object, float planeY)
{
	glm::mat4 shadowMatrix = shadowMatrixYPlane(planeY, kLightPosition);
	glm::mat4 shadowModel = shadowMatrix * modelMatrix;

	GLboolean blendEnabled = glIsEnabled(GL_BLEND);
	GLboolean depthEnabled = glIsEnabled(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(-1.0f, -1.0f);

	glBindVertexArray(object.vao);
	glUseProgram(object.program);

	glUniformMatrix4fv(object.modelLocation, 1, GL_FALSE, &shadowModel[0][0]);
	glUniformMatrix4fv(object.viewLocation, 1, GL_FALSE, &camera->viewMatrix[0][0]);
	glUniformMatrix4fv(object.projectionLocation, 1, GL_FALSE, &camera->projMatrix[0][0]);
	glUniform1i(object.shadowLocation, 1);
	glUniform1f(object.shadowAlphaLocation, object.shadowAlpha);
	glUniform1i(object.useLightingLocation, 0);
	glUniform1i(object.useTextureLocation, 0);

	glDrawArrays(GL_TRIANGLES, 0, mesh->getPoints().size());

	glDisable(GL_POLYGON_OFFSET_FILL);
	if (!blendEnabled) {
		glDisable(GL_BLEND);
	}
	if (!depthEnabled) {
		glDisable(GL_DEPTH_TEST);
	}
}

void ground_plane(glm::mat4 modelMatrix)
{
	drawScaledMesh(
		modelMatrix,
		Ground,
		GroundObject,
		glm::vec3(0.0, -0.5 * poolScene.GROUND_THICKNESS - poolScene.GROUND_DROP, 0.0),
		glm::vec3(poolScene.GROUND_SIZE, poolScene.GROUND_THICKNESS, poolScene.GROUND_SIZE));
}

void campus_ground(glm::mat4 modelMatrix)
{
	drawScaledMesh(
		modelMatrix,
		Ground,
		GroundObject,
		glm::vec3(0.0, -0.5 * poolScene.GROUND_THICKNESS - poolScene.GROUND_DROP, 0.0),
		glm::vec3(poolScene.GROUND_SIZE * kCampusScale, poolScene.GROUND_THICKNESS, poolScene.GROUND_SIZE * kCampusScale));
}

void campus_walls(glm::mat4 modelMatrix)
{
	float campusHalf = poolScene.GROUND_SIZE * kCampusScale * 0.5f;
	float groundTopY = -poolScene.GROUND_DROP;
	float wallCenterY = groundTopY + 0.5f * kCampusWallHeight;
	float wallLength = campusHalf * 2.0f + kCampusWallThickness * 2.0f;
	openGLObject wallObject = CampusWallObject;
	wallObject.texScale = glm::vec2(wallLength / 8.0f, kCampusWallHeight / 8.0f);

	drawScaledMesh(
		modelMatrix,
		CampusWall,
		wallObject,
		glm::vec3(0.0f, wallCenterY, campusHalf + 0.5f * kCampusWallThickness),
		glm::vec3(wallLength, kCampusWallHeight, kCampusWallThickness));

	drawScaledMesh(
		modelMatrix,
		CampusWall,
		wallObject,
		glm::vec3(0.0f, wallCenterY, -campusHalf - 0.5f * kCampusWallThickness),
		glm::vec3(wallLength, kCampusWallHeight, kCampusWallThickness));

	drawScaledMesh(
		modelMatrix,
		CampusWall,
		wallObject,
		glm::vec3(campusHalf + 0.5f * kCampusWallThickness, wallCenterY, 0.0f),
		glm::vec3(kCampusWallThickness, kCampusWallHeight, wallLength));

	drawScaledMesh(
		modelMatrix,
		CampusWall,
		wallObject,
		glm::vec3(-campusHalf - 0.5f * kCampusWallThickness, wallCenterY, 0.0f),
		glm::vec3(kCampusWallThickness, kCampusWallHeight, wallLength));
}

void pool_lane_floats(glm::mat4 modelMatrix)
{
	const int laneLineCount = 4;
	float lineSpacing = poolScene.POOL_WIDTH / (laneLineCount + 1.0f);
	float startZ = -poolScene.POOL_WIDTH * 0.5f + lineSpacing;
	float waterCenterY = -poolScene.WATER_THICKNESS * 0.5f - 0.05f;
	float waterSurfaceY = waterCenterY + poolScene.WATER_THICKNESS * 0.5f + 0.06f;
	float floatThickness = 0.3f;
	glm::vec3 floatScale(poolScene.POOL_LENGTH, floatThickness, floatThickness);

	for (int i = 0; i < laneLineCount; ++i) {
		float z = startZ + i * lineSpacing;
		drawScaledMesh(
			modelMatrix,
			LaneFloat,
			LaneFloatObject,
			glm::vec3(0.0f, waterSurfaceY, z),
			floatScale);
	}
}

void pool_spectator_stands(glm::mat4 modelMatrix)
{
	float groundTopY = -poolScene.GROUND_DROP;
	float wall = poolScene.WALL_THICKNESS;
	float standLength = poolScene.POOL_LENGTH + 20.0f;
	int stepCount = 5;
	float stepHeight = 10;
	float stepDepth = 30;
	float baseOffsetZ = poolScene.POOL_WIDTH * 0.5f + wall + 2.0f;
	float robotScale = 2.5f;

	for (int side = 0; side < 2; ++side) {
		float zSign = side == 0 ? 1.0f : -1.0f;
		int cols = 12;
		float spanX = poolScene.POOL_LENGTH * 0.85f;
		float startX = -spanX * 0.5f;
		float colStep = spanX / static_cast<float>(cols - 1);

		for (int step = 0; step < stepCount; ++step) {
			float stepCenterY = groundTopY + stepHeight * 0.5f + stepHeight * step;
			float stepCenterZ = baseOffsetZ + stepDepth * (step + 0.5f);
			drawScaledMesh(
				modelMatrix,
				SpectatorStand,
				SpectatorStandObject,
				glm::vec3(0.0f, stepCenterY, zSign * stepCenterZ),
				glm::vec3(standLength, stepHeight, stepDepth));

			float stepTopY = groundTopY + stepHeight * (step + 1.0f);
			float rowZ = stepCenterZ - stepDepth * 0.35f;
			for (int col = 0; col < cols; ++col) {
				unsigned int seed = static_cast<unsigned int>(side * 100000 + step * 1000 + col);
				float jitterX = (hash01(seed + 5u) - 0.5f) * 0.6f;
				float jitterZ = (hash01(seed + 11u) - 0.5f) * 0.4f;
				float staggerX = (col % 2 == 0) ? -0.2f : 0.2f;
				float staggerZ = (step % 2 == 0) ? -0.15f : 0.15f;
				float x = startX + col * colStep + jitterX + staggerX;
				float z = rowZ + jitterZ + staggerZ;
				glm::vec3 tint(
					0.3f + 0.7f * hash01(seed + 17u),
					0.3f + 0.7f * hash01(seed + 23u),
					0.3f + 0.7f * hash01(seed + 31u));
				glm::vec3 worldPos(x, stepTopY + 0.02f, zSign * z);
				glm::vec3 toPlayer = glm::vec3(gRobotPosition.x - worldPos.x, 0.0f, gRobotPosition.z - worldPos.z);
				float yaw = 0.0f;
				if (glm::length(toPlayer) > 0.001f) {
					yaw = glm::degrees(std::atan2(toPlayer.x, -toPlayer.z));
				}
				float cheerPhase = static_cast<float>(glfwGetTime()) * 4.0f + hash01(seed + 41u) * 6.28318f;
				float cheer = std::sin(cheerPhase);
				float upperArmAngle = 60.0f + 25.0f * cheer;
				float lowerArmAngle = 20.0f + 15.0f * cheer;

				glm::mat4 robotMatrix = glm::translate(modelMatrix, worldPos);
				robotMatrix = glm::rotate(robotMatrix, glm::radians(yaw), glm::vec3(0.0f, 1.0f, 0.0f));
				robotMatrix = glm::scale(robotMatrix, glm::vec3(robotScale));
				drawSpectatorRobot(robotMatrix, tint, -upperArmAngle, -lowerArmAngle, groundTopY, true);
			}
		}
	}
}

void drawAiSwimmers(glm::mat4 modelMatrix)
{
	if (gAiSwimmers.empty()) {
		return;
	}
	float shadowPlaneY = getGroundTopY();
	float basePhase = static_cast<float>(glfwGetTime()) * 3.0f;
	for (const auto& swimmer : gAiSwimmers) {
		float phaseOffset = hash01(static_cast<unsigned int>(swimmer.laneIndex * 97 + 13)) * 6.28318f;
		float phase = basePhase + phaseOffset;
		float swimArmSwing = std::sin(phase) * 35.0f;
		float swimLowerArmSwing = std::sin(phase + 0.8f) * 20.0f;
		float swimLegSwing = std::sin(phase + 3.1415926f) * 25.0f;
		float swimLowerLegSwing = std::sin(phase + 4.2f) * 15.0f;
		float swimBodyPitch = -90.0f;

		glm::mat4 swimmerMatrix = glm::translate(modelMatrix, swimmer.position);
		swimmerMatrix = glm::rotate(swimmerMatrix, glm::radians(kRobotFacingYaw), glm::vec3(0.0f, 1.0f, 0.0f));
		drawSwimmerRobot(swimmerMatrix, swimmer.tint, swimArmSwing, swimLowerArmSwing, swimLegSwing, swimLowerLegSwing, swimBodyPitch, shadowPlaneY, true);
	}
}

void pool_basin(glm::mat4 modelMatrix)
{
	float wall = poolScene.WALL_THICKNESS;
	float wallHeight = poolScene.POOL_DEPTH;
	float wallCenterY = -poolScene.POOL_DEPTH * 0.5;
	float bottomCenterY = -poolScene.POOL_DEPTH - wall * 0.5;
	float longLength = poolScene.POOL_LENGTH + 2.0 * wall;
	float shortWidth = poolScene.POOL_WIDTH + 2.0 * wall;
	float wallZ = poolScene.POOL_WIDTH * 0.5 + wall * 0.5;
	float wallX = poolScene.POOL_LENGTH * 0.5 + wall * 0.5;

	drawScaledMesh(
		modelMatrix,
		PoolWall,
		PoolWallObject,
		glm::vec3(0.0, wallCenterY, wallZ),
		glm::vec3(longLength, wallHeight, wall));
	drawScaledMesh(
		modelMatrix,
		PoolWall,
		PoolWallObject,
		glm::vec3(0.0, wallCenterY, -wallZ),
		glm::vec3(longLength, wallHeight, wall));

	drawScaledMesh(
		modelMatrix,
		PoolWall,
		PoolWallObject,
		glm::vec3(wallX, wallCenterY, 0.0),
		glm::vec3(wall, wallHeight, shortWidth));
	drawScaledMesh(
		modelMatrix,
		PoolWall,
		PoolWallObject,
		glm::vec3(-wallX, wallCenterY, 0.0),
		glm::vec3(wall, wallHeight, shortWidth));

	float waterCenterY = -poolScene.WATER_THICKNESS * 0.5 - 0.05;
	openGLObject waterObject = PoolWaterObject;
	float waterTime = static_cast<float>(glfwGetTime());
	float waterOffsetV = std::fmod(waterTime * 0.05f, 1.0f);
	waterObject.texOffset = glm::vec2(waterOffsetV,0.0f);
	drawScaledMesh(
		modelMatrix,
		PoolWater,
		waterObject,
		glm::vec3(0.0, waterCenterY, 0.0),
		glm::vec3(poolScene.POOL_LENGTH, poolScene.WATER_THICKNESS, poolScene.POOL_WIDTH));

	pool_lane_floats(modelMatrix);
}

void pool_ladder_rail(glm::mat4 modelMatrix)
{
	drawScaledMesh(
		modelMatrix,
		Ladder,
		LadderObject,
		glm::vec3(0.0, -0.5 * poolScene.LADDER_HEIGHT, 0.0),
		glm::vec3(poolScene.LADDER_RAIL_THICKNESS, poolScene.LADDER_HEIGHT, poolScene.LADDER_RAIL_THICKNESS));
}

void pool_ladder_rung(glm::mat4 modelMatrix)
{
	float rungWidth = poolScene.LADDER_WIDTH - poolScene.LADDER_RAIL_THICKNESS;
	drawScaledMesh(
		modelMatrix,
		Ladder,
		LadderObject,
		glm::vec3(-0.5 * poolScene.LADDER_RUNG_DEPTH, 0.0, 0.0),
		glm::vec3(poolScene.LADDER_RUNG_DEPTH, poolScene.LADDER_RUNG_THICKNESS, rungWidth));
}

void pool_ladder(glm::mat4 modelMatrix)
{
	MatrixStack mstack;

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, 0.0, -0.5 * poolScene.LADDER_WIDTH));
	pool_ladder_rail(modelMatrix);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, 0.0, 0.5 * poolScene.LADDER_WIDTH));
	pool_ladder_rail(modelMatrix);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -poolScene.LADDER_RUNG_START, 0.0));
	for (int i = 0; i < poolScene.LADDER_RUNG_COUNT; ++i) {
		glm::mat4 rungMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -i * poolScene.LADDER_RUNG_SPACING, 0.0));
		pool_ladder_rung(rungMatrix);
	}
	modelMatrix = mstack.pop();
}

void pool_scene(glm::mat4 modelMatrix)
{
	MatrixStack mstack;
	modelMatrix = glm::translate(modelMatrix, poolScene.position);
	mstack.push(modelMatrix);

	float wall = poolScene.WALL_THICKNESS;
	float border = poolScene.DECK_BORDER;
	float deckThickness = poolScene.DECK_THICKNESS;
	float deckY = -0.5f * deckThickness + 0.01f;
	float innerLength = poolScene.POOL_LENGTH + 2.0 * wall;
	float innerWidth = poolScene.POOL_WIDTH + 2.0 * wall;
	float deckX = innerLength * 0.5 + border * 0.5;

	pool_basin(modelMatrix);
	drawScaledMesh(
		modelMatrix,
		Deck,
		DeckObject,
		glm::vec3(deckX, deckY, 0.0f),
		glm::vec3(border, deckThickness, innerWidth));
	drawScaledMesh(
		modelMatrix,
		Deck,
		DeckObject,
		glm::vec3(-deckX, deckY, 0.0f),
		glm::vec3(border, deckThickness, innerWidth));
	pool_spectator_stands(modelMatrix);

	glm::mat4 ladderMatrix = glm::translate(
		modelMatrix,
		glm::vec3(poolScene.POOL_LENGTH * 0.5 - wall * 0.5, 0.0, -poolScene.POOL_WIDTH * 0.25));
	pool_ladder(ladderMatrix);
	float groundTopY = -poolScene.GROUND_DROP;
	drawShadowMesh(ladderMatrix, Ladder, LadderObject, groundTopY);

	modelMatrix = mstack.pop();
}

void school_building(glm::mat4 modelMatrix, const glm::vec3& size)
{
	float roofHeight = size.y * 0.2f;
	float doorHeight = size.y * 0.45f;
	float doorWidth = size.x * 0.25f;
	float doorDepth = size.z * 0.08f;
	float windowHeight = size.y * 0.2f;
	float windowWidth = size.x * 0.2f;
	float windowDepth = size.z * 0.05f;
	float windowYOffset = size.y * 0.6f;
	float windowXOffset = size.x * 0.35f;
	float frontZ = 0.5f * size.z;

	drawScaledMesh(
		modelMatrix,
		SchoolBuilding,
		SchoolBuildingObject,
		glm::vec3(0.0f, 0.5f * size.y, 0.0f),
		size);

	drawScaledMesh(
		modelMatrix,
		SchoolRoof,
		SchoolRoofObject,
		glm::vec3(0.0f, size.y + 0.5f * roofHeight, 0.0f),
		glm::vec3(size.x * 1.05f, roofHeight, size.z * 1.05f));

	drawScaledMesh(
		modelMatrix,
		SchoolDoor,
		SchoolDoorObject,
		glm::vec3(0.0f, 0.5f * doorHeight, frontZ + 0.5f * doorDepth),
		glm::vec3(doorWidth, doorHeight, doorDepth));

	glm::vec3 windowScale(windowWidth, windowHeight, windowDepth);
	glm::vec3 windowTranslate(0.0f, windowYOffset, frontZ + 0.5f * windowDepth);
	MatrixStack mstack;
	mstack.push(modelMatrix);
	glm::mat4 windowMatrix = glm::translate(modelMatrix, glm::vec3(-windowXOffset, 0.0f, 0.0f));
	drawScaledMesh(windowMatrix, SchoolWindow, SchoolWindowObject, windowTranslate, windowScale);
	modelMatrix = mstack.pop();

	mstack.push(modelMatrix);
	windowMatrix = glm::translate(modelMatrix, glm::vec3(windowXOffset, 0.0f, 0.0f));
	drawScaledMesh(windowMatrix, SchoolWindow, SchoolWindowObject, windowTranslate, windowScale);
	modelMatrix = mstack.pop();
}

void school_complex(glm::mat4 modelMatrix)
{
	MatrixStack mstack;
	mstack.push(modelMatrix);

	school_building(modelMatrix, glm::vec3(40.0f, 18.0f, 16.0f));

	glm::mat4 baseMatrix = modelMatrix;

	mstack.push(baseMatrix);
	modelMatrix = glm::translate(baseMatrix, glm::vec3(-30.0f, 0.0f, 10.0f));
	school_building(modelMatrix, glm::vec3(22.0f, 12.0f, 12.0f));
	modelMatrix = mstack.pop();

	mstack.push(baseMatrix);
	modelMatrix = glm::translate(baseMatrix, glm::vec3(30.0f, 0.0f, 10.0f));
	school_building(modelMatrix, glm::vec3(22.0f, 12.0f, 12.0f));
	modelMatrix = mstack.pop();

	mstack.push(baseMatrix);
	modelMatrix = glm::translate(baseMatrix, glm::vec3(0.0f, 0.0f, -10.0f));
	school_building(modelMatrix, glm::vec3(16.0f, 26.0f, 10.0f));
	modelMatrix = mstack.pop();
}

void swim_venue_scene(glm::mat4 modelMatrix)
{
	pool_scene(modelMatrix);
}

void appendTextureLog(const std::string& message)
{
	std::ofstream logFile(kTextureLogPath, std::ios::app);
	if (logFile.is_open()) {
		logFile << message << std::endl;
	}
}

GLuint loadTexture2D(const std::string& filename)
{
	std::string resolvedPath = filename;
#ifdef _WIN32
	std::wstring wideFile = normalizeSeparators(utf8ToWide(filename));
	std::wstring exeDir = getExeDir();
	std::wstring candidates[] = {
		wideFile,
		exeDir + L"\\" + wideFile,
		exeDir + L"\\..\\" + wideFile,
		exeDir + L"\\..\\..\\" + wideFile,
		exeDir + L"\\..\\..\\..\\" + wideFile
	};
	for (const std::wstring& candidate : candidates) {
		if (fileExistsWide(candidate)) {
			resolvedPath = wideToUtf8(candidate);
			break;
		}
	}
	appendTextureLog("Exe dir: " + wideToUtf8(exeDir));
#endif
	appendTextureLog("Try load texture: " + resolvedPath);
	int width = 0;
	int height = 0;
	int channels = 0;
	unsigned char* data = stbi_load(resolvedPath.c_str(), &width, &height, &channels, 0);
	if (!data) {
		std::stringstream ss;
		ss << "Failed to load texture: " << resolvedPath;
		const char* reason = stbi_failure_reason();
		if (reason) {
			ss << " (" << reason << ")";
		}
		appendTextureLog(ss.str());
		if (gWindow) {
			glfwSetWindowTitle(gWindow, "Texture load failed");
		}
		return 0;
	}
	std::stringstream ss;
	ss << "Loaded texture: " << resolvedPath << " (" << width << "x" << height << ")";
	appendTextureLog(ss.str());
	if (gWindow) {
		glfwSetWindowTitle(gWindow, "Texture loaded");
	}

	GLenum format = GL_RGB;
	if (channels == 1) {
		format = GL_RED;
	}
	else if (channels == 4) {
		format = GL_RGBA;
	}

	GLuint textureID = 0;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);

	stbi_image_free(data);
	return textureID;
}

void setObjectTexture(openGLObject& object, GLuint textureID, const glm::vec2& texScale)
{
	object.textureID = textureID;
	object.texScale = texScale;
	object.useTexture = textureID != 0 ? 1 : 0;
}

glm::vec3 getHeadForward()
{
	glm::vec3 forward(0.0f, 0.0f, -1.0f);
	glm::mat4 rotation(1.0f);
	rotation = glm::rotate(rotation, glm::radians(robot.theta[robot.Torso]), glm::vec3(0.0f, 1.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(robot.theta[robot.Head]), glm::vec3(0.0f, 1.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(gHeadPitch), glm::vec3(1.0f, 0.0f, 0.0f));
	return glm::normalize(glm::vec3(rotation * glm::vec4(forward, 0.0f)));
}

glm::vec3 getRobotForward()
{
	glm::vec3 forward(0.0f, 0.0f, -1.0f);
	glm::mat4 rotation(1.0f);
	rotation = glm::rotate(rotation, glm::radians(robot.theta[robot.Torso]), glm::vec3(0.0f, 1.0f, 0.0f));
	return glm::normalize(glm::vec3(rotation * glm::vec4(forward, 0.0f)));
}

float getGroundTopY()
{
	return -poolScene.GROUND_DROP;
}

float getRobotStandY()
{
	return getGroundTopY() + robot.UPPER_LEG_HEIGHT + robot.LOWER_LEG_HEIGHT;
}

float getCampusHalfExtent()
{
	return poolScene.GROUND_SIZE * kCampusScale * 0.5f;
}

float getPoolHalfLength()
{
	return poolScene.POOL_LENGTH * 0.5f;
}

float getPoolHalfWidth()
{
	return poolScene.POOL_WIDTH * 0.5f;
}

float getPoolStartX()
{
	return poolScene.position.x - getPoolHalfLength() + kRaceStartInset;
}

float getPoolFinishX()
{
	return poolScene.position.x + getPoolHalfLength() - kRobotCollisionRadius;
}

float getLaneWidth()
{
	return poolScene.POOL_WIDTH / static_cast<float>(kLaneCount);
}

float getLaneCenterZ(int laneIndex)
{
	float laneWidth = getLaneWidth();
	return -getPoolHalfWidth() + laneWidth * (laneIndex + 0.5f);
}

float getLaneCenterZWorld(int laneIndex)
{
	return poolScene.position.z + getLaneCenterZ(laneIndex);
}

float getRobotLaneCenterZ()
{
	return getLaneCenterZ(kRobotLaneIndex);
}

float getRobotLaneCenterZWorld()
{
	return getLaneCenterZWorld(kRobotLaneIndex);
}

float hash01(unsigned int seed)
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return static_cast<float>(seed & 0xFFFFu) / 65535.0f;
}

void resetAiSwimmers()
{
	float startX = getPoolStartX();
	float baseY = getRobotStandY();
	for (auto& swimmer : gAiSwimmers) {
		swimmer.position = glm::vec3(startX, baseY, getLaneCenterZWorld(swimmer.laneIndex));
		swimmer.finished = false;
	}
}

void initAiSwimmers()
{
	gAiSwimmers.clear();
	for (int lane = 0; lane < kLaneCount; ++lane) {
		if (lane == kRobotLaneIndex || lane == kSecondPlayerLaneIndex) {
			continue;
		}
		SwimmerState swimmer;
		swimmer.laneIndex = lane;
		unsigned int seed = static_cast<unsigned int>(lane * 1009 + 17);
		swimmer.speed = kAiMinSpeed + hash01(seed) * (kAiMaxSpeed - kAiMinSpeed);
		swimmer.tint = glm::vec3(
			0.3f + 0.7f * hash01(seed + 3u),
			0.3f + 0.7f * hash01(seed + 7u),
			0.3f + 0.7f * hash01(seed + 11u));
		gAiSwimmers.push_back(swimmer);
	}
	resetAiSwimmers();
}

void announceRaceStatus(const std::string& message)
{
	if (gWindow) {
		glfwSetWindowTitle(gWindow, message.c_str());
	}
	std::cout << message << std::endl;
}

void startRace()
{
	gRaceStarted = true;
	gRaceFinished = false;
	gPlayerFinished = false;
	gWinnerLane = -1;
	gStartRequested = false;
	if (gPlayerSwimSpeed < 0.0f) {
		gPlayerSwimSpeed = 0.0f;
	}
	if (gPlayerSwimSpeed > kPlayerMaxSpeed) {
		gPlayerSwimSpeed = kPlayerMaxSpeed;
	}
	gSecondSwimSpeed = 0.0f;
	gSecondFinished = false;
	resetAiSwimmers();
	announceRaceStatus("Race started");
}

void finishRace(const std::string& message, int winnerLane)
{
	gRaceFinished = true;
	gWinnerLane = winnerLane;
	announceRaceStatus(message);
}

void resetRace()
{
	gRaceStarted = false;
	gRaceFinished = false;
	gPlayerFinished = false;
	gSecondFinished = false;
	gWinnerLane = -1;
	gStartRequested = false;
	gPlayerSwimSpeed = 0.0f;
	gRobotVelocityY = 0.0f;
	gRobotPosition = glm::vec3(getPoolStartX(), getRobotStandY(), getRobotLaneCenterZWorld());
	gSecondSwimSpeed = 0.0f;
	gSecondRobotPosition = glm::vec3(getPoolStartX(), getRobotStandY(), getLaneCenterZWorld(kSecondPlayerLaneIndex));
	robot.theta[robot.Torso] = kRobotFacingYaw;
	gPlayerYaw = kRobotFacingYaw;
	gSecondYaw = kRobotFacingYaw;
	gPlayerYawAccum = 0.0f;
	gSecondYawAccum = 0.0f;
	gPlayerScale = 1.0f;
	gSecondScale = 1.0f;
	resetAiSwimmers();
	announceRaceStatus("Press D to start");
}

bool isRobotInPool(const glm::vec3& position)
{
	glm::vec3 local = position - poolScene.position;
	float halfLength = poolScene.POOL_LENGTH * 0.5f - kRobotCollisionRadius;
	float halfWidth = poolScene.POOL_WIDTH * 0.5f - kRobotCollisionRadius;
	return std::abs(local.x) <= halfLength && std::abs(local.z) <= halfWidth;
}

void updateRaceState(float deltaTime)
{
	if (!gRaceStarted && !gRaceFinished) {
		if (gStartRequested && isRobotInPool(gRobotPosition)) {
			startRace();
		}
	}
	if (!gRaceStarted || gRaceFinished) {
		return;
	}

	float finishX = getPoolFinishX();
	if (!gPlayerFinished && gRobotPosition.x >= finishX) {
		gPlayerFinished = true;
		finishRace("Winner: Player", kRobotLaneIndex);
		return;
	}

	if (!gSecondFinished && gSecondRobotPosition.x >= finishX) {
		gSecondFinished = true;
		finishRace("Winner: Player 2", kSecondPlayerLaneIndex);
		return;
	}

	for (auto& swimmer : gAiSwimmers) {
		if (swimmer.finished) {
			continue;
		}
		swimmer.position.x += swimmer.speed * deltaTime;
		if (swimmer.position.x >= finishX) {
			swimmer.position.x = finishX;
			swimmer.finished = true;
			if (!gRaceFinished) {
				std::stringstream ss;
				ss << "Winner: Lane " << (swimmer.laneIndex + 1);
				finishRace(ss.str(), swimmer.laneIndex);
				return;
			}
		}
	}
}

void updateCameraFollow()
{
	float horizontal = (std::max)(poolScene.POOL_LENGTH, poolScene.POOL_WIDTH);
	float vertical = horizontal;
	float diag = horizontal * 0.3;
	glm::vec3 focus = poolScene.position;
	glm::vec3 baseOffset(0.0f, vertical / 3.0f, diag);
	glm::mat4 rotation(1.0f);
	rotation = glm::rotate(rotation, glm::radians(gCameraYawOffset), glm::vec3(0.0f, 1.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(gCameraPitchOffset), glm::vec3(1.0f, 0.0f, 0.0f));
	glm::vec3 cameraPosition = focus + glm::vec3(rotation * glm::vec4(baseOffset, 0.0f));
	camera->eye = glm::vec4(cameraPosition, 1.0f);
	camera->at = glm::vec4(focus, 1.0f);
	camera->up = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
}

void drawSkybox()
{
	GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
	GLboolean cullEnabled = glIsEnabled(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDepthMask(GL_FALSE);

	auto drawSkyboxFace = [&](const glm::mat4& baseMatrix, const glm::vec3& translate,
		const glm::vec3& axis, float angleDegrees, GLuint textureID) {
		setObjectTexture(SkyboxObject, textureID, glm::vec2(1.0f, 1.0f));
		SkyboxObject.texOffset = glm::vec2(0.0f, 0.0f);
		glm::mat4 instance = glm::mat4(1.0f);
		instance = glm::translate(instance, translate);
		if (angleDegrees != 0.0f) {
			instance = glm::rotate(instance, glm::radians(angleDegrees), axis);
		}
		instance = glm::scale(instance, glm::vec3(poolScene.SKYBOX_SIZE));
		drawMesh(baseMatrix * instance, Skybox, SkyboxObject);
	};

	glm::mat4 baseMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(camera->eye));
	float halfSize = 0.5f * poolScene.SKYBOX_SIZE;

	drawSkyboxFace(baseMatrix, glm::vec3(0.0f, 0.0f, -halfSize), glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, gSkyboxFrontTexture);
	drawSkyboxFace(baseMatrix, glm::vec3(0.0f, 0.0f, halfSize), glm::vec3(0.0f, 1.0f, 0.0f), 180.0f, gSkyboxBackTexture);
	drawSkyboxFace(baseMatrix, glm::vec3(halfSize, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, gSkyboxRightTexture);
	drawSkyboxFace(baseMatrix, glm::vec3(-halfSize, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 90.0f, gSkyboxLeftTexture);
	drawSkyboxFace(baseMatrix, glm::vec3(0.0f, halfSize, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), -90.0f, gSkyboxTopTexture);
	drawSkyboxFace(baseMatrix, glm::vec3(0.0f, -halfSize, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 90.0f, gSkyboxBottomTexture);

	glDepthMask(GL_TRUE);
	if (depthTestEnabled) {
		glEnable(GL_DEPTH_TEST);
	}
	if (cullEnabled) {
		glEnable(GL_CULL_FACE);
	}
}

void bindObjectAndData(TriMesh* mesh, openGLObject& object, const std::string &vshader, const std::string &fshader) {

	// 创建顶点数组对象
	glGenVertexArrays(1, &object.vao);  	// 分配1个顶点数组对象
	glBindVertexArray(object.vao);  	// 绑定顶点数组对象

	// 创建并初始化顶点缓存对象
	glGenBuffers(1, &object.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, object.vbo);
	size_t pointsSize = mesh->getPoints().size() * sizeof(glm::vec3);
	size_t colorsSize = mesh->getColors().size() * sizeof(glm::vec3);
	size_t normalsSize = mesh->getNormals().size() * sizeof(glm::vec3);
	size_t texcoordsSize = mesh->getTexCoords().size() * sizeof(glm::vec2);
	glBufferData(GL_ARRAY_BUFFER, 
		pointsSize + colorsSize + normalsSize + texcoordsSize,
		NULL, 
		GL_STATIC_DRAW);

	// 修改完TriMesh.cpp的代码成后再打开下面注释，否则程序会报错
	glBufferSubData(GL_ARRAY_BUFFER, 0, pointsSize, &mesh->getPoints()[0]);
	glBufferSubData(GL_ARRAY_BUFFER, pointsSize, colorsSize, &mesh->getColors()[0]);
	glBufferSubData(GL_ARRAY_BUFFER, pointsSize + colorsSize, normalsSize, &mesh->getNormals()[0]);
	glBufferSubData(GL_ARRAY_BUFFER, pointsSize + colorsSize + normalsSize, texcoordsSize, &mesh->getTexCoords()[0]);

	object.vshader = vshader;
	object.fshader = fshader;
	object.program = InitShader(object.vshader.c_str(), object.fshader.c_str());

	// 从顶点着色器中初始化顶点的坐标
	object.pLocation = glGetAttribLocation(object.program, "vPosition");
	glEnableVertexAttribArray(object.pLocation);
	glVertexAttribPointer(object.pLocation, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

	// 从顶点着色器中初始化顶点的颜色
	object.cLocation = glGetAttribLocation(object.program, "vColor");
	glEnableVertexAttribArray(object.cLocation);
	glVertexAttribPointer(object.cLocation, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(mesh->getPoints().size() * sizeof(glm::vec3)));

	// 从顶点着色器中初始化顶点的法向量
	object.nLocation = glGetAttribLocation(object.program, "vNormal");
	glEnableVertexAttribArray(object.nLocation);
	glVertexAttribPointer(object.nLocation, 3, 
		GL_FLOAT, GL_FALSE, 0, 
		BUFFER_OFFSET(pointsSize + colorsSize));

	object.tLocation = glGetAttribLocation(object.program, "vTexCoord");
	glEnableVertexAttribArray(object.tLocation);
	glVertexAttribPointer(object.tLocation, 2,
		GL_FLOAT, GL_FALSE, 0,
		BUFFER_OFFSET(pointsSize + colorsSize + normalsSize));


	// 获得矩阵位置
	object.modelLocation = glGetUniformLocation(object.program, "model");
	object.viewLocation = glGetUniformLocation(object.program, "view");
	object.projectionLocation = glGetUniformLocation(object.program, "projection");

	object.shadowLocation = glGetUniformLocation(object.program, "isShadow");
	object.textureLocation = glGetUniformLocation(object.program, "tex");
	object.useTextureLocation = glGetUniformLocation(object.program, "useTexture");
	object.texScaleLocation = glGetUniformLocation(object.program, "texScale");
	object.texOffsetLocation = glGetUniformLocation(object.program, "texOffset");
	object.colorTintLocation = glGetUniformLocation(object.program, "colorTint");
	object.alphaLocation = glGetUniformLocation(object.program, "alpha");
	object.useLightingLocation = glGetUniformLocation(object.program, "useLighting");
	object.lightPosLocation = glGetUniformLocation(object.program, "lightPos");
	object.lightColorLocation = glGetUniformLocation(object.program, "lightColor");
	object.ambientStrengthLocation = glGetUniformLocation(object.program, "ambientStrength");
	object.specStrengthLocation = glGetUniformLocation(object.program, "specStrength");
	object.shininessLocation = glGetUniformLocation(object.program, "shininess");
	object.eyePositionLocation = glGetUniformLocation(object.program, "eye_position");
	object.shadowAlphaLocation = glGetUniformLocation(object.program, "shadowAlpha");
}


void bindLightAndMaterial(TriMesh* mesh, openGLObject& object, Light* light, Camera* camera) {

	// 传递相机的位置
	glUniform3fv(glGetUniformLocation(object.program, "eye_position"), 1, &camera->eye[0]);

	// 传递物体的材质
	glm::vec4 meshAmbient = mesh->getAmbient();
	glm::vec4 meshDiffuse = mesh->getDiffuse();
	glm::vec4 meshSpecular = mesh->getSpecular();
	float meshShininess = mesh->getShininess();
	glUniform4fv(glGetUniformLocation(object.program, "material.ambient"), 1, &meshAmbient[0]);
	glUniform4fv(glGetUniformLocation(object.program, "material.diffuse"), 1, &meshDiffuse[0]);
	glUniform4fv(glGetUniformLocation(object.program, "material.specular"), 1, &meshSpecular[0]);
	glUniform1f(glGetUniformLocation(object.program, "material.shininess"), meshShininess);

	// 传递光源信息
	glm::vec4 lightAmbient = light->getAmbient();
	glm::vec4 lightDiffuse = light->getDiffuse();
	glm::vec4 lightSpecular = light->getSpecular();
	glm::vec3 lightPosition = light->getTranslation();

	glUniform4fv(glGetUniformLocation(object.program, "light.ambient"), 1, &lightAmbient[0]);
	glUniform4fv(glGetUniformLocation(object.program, "light.diffuse"), 1, &lightDiffuse[0]);
	glUniform4fv(glGetUniformLocation(object.program, "light.specular"), 1, &lightSpecular[0]);
	glUniform3fv(glGetUniformLocation(object.program, "light.position"), 1, &lightPosition[0]);

}


void init()
{
	std::string vshader, fshader;
	// 读取着色器并使用
	vshader = "shaders/vshader.glsl";
	fshader = "shaders/fshader.glsl";
	stbi_set_flip_vertically_on_load(true);

	gRobotPosition = glm::vec3(
		getPoolStartX(),
		getRobotStandY(),
		getRobotLaneCenterZWorld());
	gSecondRobotPosition = glm::vec3(
		getPoolStartX(),
		getRobotStandY(),
		getLaneCenterZWorld(kSecondPlayerLaneIndex));
	robot.theta[robot.Torso] = kRobotFacingYaw;
	gPlayerYaw = kRobotFacingYaw;
	gSecondYaw = kRobotFacingYaw;
	gPlayerYawAccum = 0.0f;
	gSecondYawAccum = 0.0f;
	gPlayerScale = 1.0f;
	gSecondScale = 1.0f;
	initAiSwimmers();
	gRaceStarted = false;
	gRaceFinished = false;
	gPlayerFinished = false;
	gStartRequested = false;
	gSecondFinished = false;
	gSecondSwimSpeed = 0.0f;

	// 设置物体的大小（初始的旋转和位移都为0）
	Torso->generateCube(Blue);
	Head->generateCube(Green);
	RightUpperArm->generateCube(Yellow);
	LeftUpperArm->generateCube(Yellow);
	RightUpperLeg->generateCube(Brown);
	LeftUpperLeg->generateCube(Brown);
	RightLowerArm->generateCube(Red);
	LeftLowerArm->generateCube(Red);
	RightLowerLeg->generateCube(Cyan);
	LeftLowerLeg->generateCube(Cyan);
	Ground->generateCube(Green);
	Deck->generateCube(Brown);
	PoolBottom->generateCube(Blue);
	PoolWall->generateCube(White);
	PoolWater->generateCube(Cyan);
	Ladder->generateCube(Yellow);
	Skybox->generateSquare(SkyBlue);
	SchoolBuilding->generateCube(White);
	SchoolRoof->generateCube(Red);
	SchoolDoor->generateCube(Brown);
	SchoolWindow->generateCube(Cyan);
	CampusWall->generateCube(glm::vec3(0.7f, 0.7f, 0.7f));
	SwimRing->generateCube(glm::vec3(1.0f, 0.55f, 0.1f));
	LaneFloat->generateCube(glm::vec3(1.0f, 0.6f, 0.2f));
	SpectatorStand->generateCube(glm::vec3(0.4f, 0.4f, 0.45f));
	Spectator->generateCube(glm::vec3(0.8f, 0.7f, 0.4f));
	

	// 将物体的顶点数据传递
	bindObjectAndData(Torso, TorsoObject, vshader, fshader);
	bindObjectAndData(Head, HeadObject, vshader, fshader);
	bindObjectAndData(RightUpperArm, RightUpperArmObject, vshader, fshader);
	bindObjectAndData(LeftUpperArm, LeftUpperArmObject, vshader, fshader);
	bindObjectAndData(RightUpperLeg, RightUpperLegObject, vshader, fshader);
	bindObjectAndData(LeftUpperLeg, LeftUpperLegObject, vshader, fshader);
	bindObjectAndData(RightLowerArm, RightLowerArmObject, vshader, fshader);
	bindObjectAndData(LeftLowerArm, LeftLowerArmObject, vshader, fshader);
	bindObjectAndData(RightLowerLeg, RightLowerLegObject, vshader, fshader);
	bindObjectAndData(LeftLowerLeg, LeftLowerLegObject, vshader, fshader);	
	bindObjectAndData(Ground, GroundObject, vshader, fshader);
	bindObjectAndData(Deck, DeckObject, vshader, fshader);
	bindObjectAndData(PoolBottom, PoolBottomObject, vshader, fshader);
	bindObjectAndData(PoolWall, PoolWallObject, vshader, fshader);
	bindObjectAndData(PoolWater, PoolWaterObject, vshader, fshader);
	PoolWaterObject.alpha = 0.6f;
	bindObjectAndData(Ladder, LadderObject, vshader, fshader);
	bindObjectAndData(Skybox, SkyboxObject, vshader, fshader);
	bindObjectAndData(SchoolBuilding, SchoolBuildingObject, vshader, fshader);
	bindObjectAndData(SchoolRoof, SchoolRoofObject, vshader, fshader);
	bindObjectAndData(SchoolDoor, SchoolDoorObject, vshader, fshader);
	bindObjectAndData(SchoolWindow, SchoolWindowObject, vshader, fshader);
	bindObjectAndData(CampusWall, CampusWallObject, vshader, fshader);
	bindObjectAndData(SwimRing, SwimRingObject, vshader, fshader);
	bindObjectAndData(LaneFloat, LaneFloatObject, vshader, fshader);
	bindObjectAndData(SpectatorStand, SpectatorStandObject, vshader, fshader);
	bindObjectAndData(Spectator, SpectatorObject, vshader, fshader);
	SkyboxObject.useLighting = 0;

	if (kEnableTextures) {
		gSkyboxFrontTexture = loadTexture2D(u8"assets/skybox_front.jpg");
		gSkyboxBackTexture = loadTexture2D(u8"assets/skybox_back.jpg");
		gSkyboxLeftTexture = loadTexture2D(u8"assets/skybox_left.jpg");
		gSkyboxRightTexture = loadTexture2D(u8"assets/skybox_right.jpg");
		gSkyboxTopTexture = loadTexture2D(u8"assets/skybox_top.jpg");
		gSkyboxBottomTexture = loadTexture2D(u8"assets/skybox_bottom.jpg");
		GLuint tileTexture = loadTexture2D(u8"assets/pool_ground.jpg");
		GLuint waterTexture = loadTexture2D(u8"assets/water.jpg");
		GLuint wallTexture = loadTexture2D(u8"assets/walltexture.jpg");
		setObjectTexture(SkyboxObject, gSkyboxFrontTexture, glm::vec2(1.0f, 1.0f));
		setObjectTexture(GroundObject, tileTexture, glm::vec2(6.0f, 6.0f));
		setObjectTexture(DeckObject, tileTexture, glm::vec2(12.0f, 12.0f));
		setObjectTexture(PoolWallObject, tileTexture, glm::vec2(4.0f, 2.0f));
		setObjectTexture(PoolBottomObject, tileTexture, glm::vec2(4.0f, 2.0f));
		setObjectTexture(PoolWaterObject, waterTexture, glm::vec2(3.0f, 3.0f));
		setObjectTexture(CampusWallObject, wallTexture, glm::vec2(2.0f, 2.0f));
	}
	
	glClearColor(0.25f, 0.6f, 0.9f, 1.0f);
	announceRaceStatus("Press D to start");
}



void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 相机矩阵计算
	updateCameraFollow();
	camera->viewMatrix = camera->getViewMatrix();
	camera->projMatrix = camera->getProjectionMatrix(false);

	drawSkybox();

	// 物体的变换矩阵
	glm::mat4 modelMatrix = glm::mat4(1.0);

	// 保持变换矩阵的栈
	MatrixStack mstack;

	float groundTopY = -poolScene.GROUND_DROP;
	glm::vec3 robotBase = gRobotPosition;
	bool inPool = isRobotInPool(robotBase);
	float swimPhase = static_cast<float>(glfwGetTime()) * 3.0f;
	float swimArmSwing = inPool ? std::sin(swimPhase) * 35.0f : 0.0f;
	float swimLowerArmSwing = inPool ? std::sin(swimPhase + 0.8f) * 20.0f : 0.0f;
	float swimLegSwing = inPool ? std::sin(swimPhase + 3.1415926f) * 25.0f : 0.0f;
	float swimLowerLegSwing = inPool ? std::sin(swimPhase + 4.2f) * 15.0f : 0.0f;
	float swimBodyPitch = inPool ? -90.0f : 0.0f;

	robot.theta[robot.Torso] = gPlayerYaw;
    // 躯干（这里我们希望机器人的躯干只绕Y轴旋转，所以只计算了RotateY）
	modelMatrix = glm::translate(modelMatrix, robotBase);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(gPlayerScale));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Torso]), glm::vec3(0.0, 1.0, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(swimBodyPitch), glm::vec3(1.0, 0.0, 0.0));
	torso(modelMatrix, groundTopY, true);

	mstack.push(modelMatrix); // 保存躯干变换矩阵
    // 头部（这里我们希望机器人的头部只绕Y轴旋转，所以只计算了RotateY）
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, robot.TORSO_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Head]), glm::vec3(0.0, 1.0, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(gHeadPitch), glm::vec3(1.0, 0.0, 0.0));
	head(modelMatrix, groundTopY, true);
	modelMatrix = mstack.pop(); // 恢复躯干变换矩阵


    // =========== 左臂 ===========
	mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // 左大臂（这里我们希望机器人的左大臂只绕Z轴旋转，所以只计算了RotateZ，后面同理）
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5 * robot.TORSO_WIDTH - 0.5 * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperArm] + swimArmSwing), glm::vec3(0.0, 0.0, 1.0));
	left_upper_arm(modelMatrix, groundTopY, true);

    // @TODO: 左小臂
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerArm] + swimLowerArmSwing), glm::vec3(0.0, 0.0, 1.0));
	left_lower_arm(modelMatrix, groundTopY, true);
	modelMatrix = mstack.pop();   // 恢复躯干变换矩阵


    // =========== 右臂 ===========

	// @TODO: 右大臂
	mstack.push(modelMatrix);   // 保存躯干变换矩阵
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5 * robot.TORSO_WIDTH + 0.5 * robot.UPPER_ARM_WIDTH, robot.TORSO_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperArm] - swimArmSwing), glm::vec3(0.0, 0.0, 1.0));
	right_upper_arm(modelMatrix, groundTopY, true);



    // @TODO: 右小臂
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerArm] - swimLowerArmSwing), glm::vec3(0.0, 0.0, 1.0));
	right_lower_arm(modelMatrix, groundTopY, true);

	// 游泳圈
	glm::mat4 ringMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, -robot.LOWER_ARM_HEIGHT - 0.2f, 0.0f));
	ringMatrix = glm::rotate(ringMatrix, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	swim_ring(ringMatrix, groundTopY, true);
	modelMatrix = mstack.pop();   // 恢复躯干变换矩阵


    // =========== 左腿 ===========
	
    // @TODO: 左大腿
	mstack.push(modelMatrix);   // 保存躯干变换矩阵
	modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.5 * robot.TORSO_WIDTH + 0.5 * robot.UPPER_LEG_WIDTH, 0.0, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperLeg] + swimLegSwing), glm::vec3(1.0, 0.0, 0.0));
	left_upper_leg(modelMatrix, groundTopY, true);




	// @TODO: 左小腿
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_LEG_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerLeg] + swimLowerLegSwing), glm::vec3(1.0, 0.0, 0.0));
	left_lower_leg(modelMatrix, groundTopY, true);
	modelMatrix = mstack.pop();   // 恢复躯干变换矩阵




    // =========== 右腿 ===========
	
    // @TODO: 右大腿
	mstack.push(modelMatrix);   // 保存躯干变换矩阵
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.5 * robot.TORSO_WIDTH - 0.5 * robot.UPPER_LEG_WIDTH, 0.0, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperLeg] - swimLegSwing), glm::vec3(1.0, 0.0, 0.0));
	right_upper_leg(modelMatrix, groundTopY, true);



    // @TODO: 右小腿
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_LEG_HEIGHT, 0.0));
	modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerLeg] - swimLowerLegSwing), glm::vec3(1.0, 0.0, 0.0));
	right_lower_leg(modelMatrix, groundTopY, true);
	modelMatrix = mstack.pop();   // 恢复躯干变换矩阵

	bool secondInPool = isRobotInPool(gSecondRobotPosition);
	float secondPhase = static_cast<float>(glfwGetTime()) * 3.0f + 1.4f;
	float secondArmSwing = secondInPool ? std::sin(secondPhase) * 35.0f : 0.0f;
	float secondLowerArmSwing = secondInPool ? std::sin(secondPhase + 0.8f) * 20.0f : 0.0f;
	float secondLegSwing = secondInPool ? std::sin(secondPhase + 3.1415926f) * 25.0f : 0.0f;
	float secondLowerLegSwing = secondInPool ? std::sin(secondPhase + 4.2f) * 15.0f : 0.0f;
	float secondBodyPitch = secondInPool ? -90.0f : 0.0f;
	glm::mat4 secondMatrix = glm::translate(glm::mat4(1.0f), gSecondRobotPosition);
	secondMatrix = glm::scale(secondMatrix, glm::vec3(gSecondScale));
	secondMatrix = glm::rotate(secondMatrix, glm::radians(gSecondYaw), glm::vec3(0.0f, 1.0f, 0.0f));
	drawSwimmerRobot(secondMatrix, glm::vec3(0.2f, 0.8f, 0.9f), secondArmSwing, secondLowerArmSwing, secondLegSwing, secondLowerLegSwing, secondBodyPitch, groundTopY, true);

	glm::vec3 labelOffset(0.0f, robot.TORSO_HEIGHT + robot.HEAD_HEIGHT + 0.8f, 0.0f);
	drawPlayerNumber(glm::mat4(1.0f), gRobotPosition + labelOffset * gPlayerScale, 1, glm::vec3(1.0f, 0.9f, 0.2f));
	drawPlayerNumber(glm::mat4(1.0f), gSecondRobotPosition + labelOffset * gSecondScale, 2, glm::vec3(0.2f, 0.9f, 1.0f));

	
	modelMatrix = glm::mat4(1.0);
	drawAiSwimmers(modelMatrix);
	swim_venue_scene(modelMatrix);


}


void printHelp()
{

	std::cout << "================================================" << std::endl << std::endl;
	std::cout << "Use right click to open Menu." << std::endl;
	std::cout << "================================================" << std::endl << std::endl;

	std::cout << "Keyboard Usage" << std::endl;
	std::cout <<
		"[Window]" << std::endl <<
		"ESC:		Exit" << std::endl <<
		"h:		Print help message" << std::endl <<
		std::endl <<

		"[Part]" << std::endl <<
		"1:		Torso" << std::endl <<
		"2:		Head" << std::endl <<
		"3:		RightUpperArm" << std::endl <<
		"4:		RightLowerArm" << std::endl <<
		"5:		LeftUpperArm" << std::endl <<
		"6:		LeftLowerArm" << std::endl <<
		"7:		RightUpperLeg" << std::endl <<
		"8:		RightLowerLeg" << std::endl <<
		"9:		LeftUpperLeg" << std::endl <<
		"0:		LeftLowerLeg" << std::endl <<
		std::endl <<

		"[Model]" << std::endl <<
		"LEFT/RIGHT:	Rotate selected part" << std::endl <<

		std::endl <<
		"[Robot]" << std::endl <<
		"D:		Start race" << std::endl <<
		"A/D:		Stroke to increase swim speed" << std::endl <<
		"LEFT/RIGHT:	Stroke for player 2" << std::endl <<
		"P:		Restart after finish" << std::endl <<

		std::endl <<
		"[Camera]" << std::endl <<
		"Mouse drag: rotate view" << std::endl << std::endl;

}


// 键盘响应函数
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	float tmp;
	glm::vec4 ambient;
	if (action == GLFW_PRESS) {
		switch (key)
		{
		case GLFW_KEY_ESCAPE: exit(EXIT_SUCCESS); break;
		case GLFW_KEY_Q: exit(EXIT_SUCCESS); break;
		case GLFW_KEY_P:
			if (gRaceFinished) {
				resetRace();
			}
			break;
		case GLFW_KEY_D:
			if (!gRaceStarted && !gRaceFinished) {
				gStartRequested = true;
				gPlayerSwimSpeed = kPlayerStrokeBoost;
			}
			else if (!gRaceFinished) {
				gPlayerSwimSpeed = (std::min)(gPlayerSwimSpeed + kPlayerStrokeBoost, kPlayerMaxSpeed);
			}
			break;
		case GLFW_KEY_A:
			if (gRaceStarted && !gRaceFinished) {
				gPlayerSwimSpeed = (std::min)(gPlayerSwimSpeed + kPlayerStrokeBoost, kPlayerMaxSpeed);
			}
			break;
		case GLFW_KEY_W:
			gPlayerYaw += kPlayerYawStep;
			if (gPlayerYaw >= 360.0f) {
				gPlayerYaw -= 360.0f;
			}
			gPlayerYawAccum += kPlayerYawStep;
			if (gPlayerYawAccum >= 360.0f) {
				gPlayerYawAccum -= 360.0f;
				gPlayerScale *= 1.5f;
			}
			break;
		case GLFW_KEY_S:
			gPlayerYaw -= kPlayerYawStep;
			if (gPlayerYaw < 0.0f) {
				gPlayerYaw += 360.0f;
			}
			gPlayerYawAccum += kPlayerYawStep;
			if (gPlayerYawAccum >= 360.0f) {
				gPlayerYawAccum -= 360.0f;
				gPlayerScale *= 1.5f;
			}
			break;
		case GLFW_KEY_UP:
			gSecondYaw += kPlayerYawStep;
			if (gSecondYaw >= 360.0f) {
				gSecondYaw -= 360.0f;
			}
			gSecondYawAccum += kPlayerYawStep;
			if (gSecondYawAccum >= 360.0f) {
				gSecondYawAccum -= 360.0f;
				gSecondScale *= 1.5f;
			}
			break;
		case GLFW_KEY_DOWN:
			gSecondYaw -= kPlayerYawStep;
			if (gSecondYaw < 0.0f) {
				gSecondYaw += 360.0f;
			}
			gSecondYawAccum += kPlayerYawStep;
			if (gSecondYawAccum >= 360.0f) {
				gSecondYawAccum -= 360.0f;
				gSecondScale *= 1.5f;
			}
			break;
		case GLFW_KEY_1: Selected_mesh = robot.Torso; break;
		case GLFW_KEY_2: Selected_mesh = robot.Head; break;
		case GLFW_KEY_3: Selected_mesh = robot.RightUpperArm; break;
		case GLFW_KEY_4: Selected_mesh = robot.RightLowerArm; break;
		case GLFW_KEY_5: Selected_mesh = robot.LeftUpperArm; break;
		case GLFW_KEY_6: Selected_mesh = robot.LeftLowerArm; break;
		case GLFW_KEY_7: Selected_mesh = robot.RightUpperLeg; break;
		case GLFW_KEY_8: Selected_mesh = robot.RightLowerLeg; break;
		case GLFW_KEY_9: Selected_mesh = robot.LeftUpperLeg; break;
		case GLFW_KEY_0: Selected_mesh = robot.LeftLowerLeg; break;
		case GLFW_KEY_LEFT:
		case GLFW_KEY_RIGHT:
			if (gRaceStarted && !gRaceFinished) {
				gSecondSwimSpeed = (std::min)(gSecondSwimSpeed + kPlayerStrokeBoost, kPlayerMaxSpeed);
			}
			else {
				if (key == GLFW_KEY_LEFT) {
					robot.theta[Selected_mesh] -= 5.0;
					if (robot.theta[Selected_mesh] < 0.0)
						robot.theta[Selected_mesh] += 360.0;
				}
				else {
					robot.theta[Selected_mesh] += 5.0;
					if (robot.theta[Selected_mesh] > 360.0)
						robot.theta[Selected_mesh] -= 360.0;
				}
			}
			break;
		case GLFW_KEY_SPACE:
			gCameraYawOffset = 0.0f;
			gCameraPitchOffset = 0.0f;
			gFirstMouse = true;
			break;
		default:
			break;
		}
	}
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (!gIsDragging) {
		gLastX = xpos;
		gLastY = ypos;
		return;
	}

	if (gFirstMouse) {
		gLastX = xpos;
		gLastY = ypos;
		gFirstMouse = false;
		return;
	}

	float xoffset = static_cast<float>(xpos - gLastX);
	float yoffset = static_cast<float>(gLastY - ypos);
	gLastX = xpos;
	gLastY = ypos;

	gCameraYawOffset += xoffset * gMouseSensitivity;
	if (gCameraYawOffset >= 360.0f) {
		gCameraYawOffset -= 360.0f;
	}
	if (gCameraYawOffset < 0.0f) {
		gCameraYawOffset += 360.0f;
	}
	gCameraPitchOffset += yoffset * gMouseSensitivity;
	if (gCameraPitchOffset > 60.0f) {
		gCameraPitchOffset = 60.0f;
	}
	if (gCameraPitchOffset < -60.0f) {
		gCameraPitchOffset = -60.0f;
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			gIsDragging = true;
			gFirstMouse = true;
		}
		else if (action == GLFW_RELEASE) {
			gIsDragging = false;
		}
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	gCameraFollowDistance -= static_cast<float>(yoffset) * 2.0f;
	if (gCameraFollowDistance < 3.0f) {
		gCameraFollowDistance = 3.0f;
	}
	if (gCameraFollowDistance > 60.0f) {
		gCameraFollowDistance = 60.0f;
	}
}

void processMovement(GLFWwindow* window, float deltaTime)
{
	(void)window;
	if (!gRaceStarted) {
		if (!gStartRequested) {
			gPlayerSwimSpeed = 0.0f;
		}
	}
	else if (gRaceFinished) {
		gPlayerSwimSpeed = 0.0f;
	}
	else {
		gPlayerSwimSpeed = (std::max)(0.0f, gPlayerSwimSpeed - kPlayerSpeedDecay * deltaTime);
		gRobotPosition.x += gPlayerSwimSpeed * gPlayerScale * deltaTime;
	}

	if (!gRaceStarted || gRaceFinished) {
		gSecondSwimSpeed = 0.0f;
	}
	else {
		gSecondSwimSpeed = (std::max)(0.0f, gSecondSwimSpeed - kPlayerSpeedDecay * deltaTime);
		gSecondRobotPosition.x += gSecondSwimSpeed * gSecondScale * deltaTime;
	}

	float minX = poolScene.position.x - getPoolHalfLength() + kRobotCollisionRadius;
	float maxX = getPoolFinishX();
	if (gRobotPosition.x < minX) {
		gRobotPosition.x = minX;
	}
	if (gRobotPosition.x > maxX) {
		gRobotPosition.x = maxX;
	}
	if (gSecondRobotPosition.x < minX) {
		gSecondRobotPosition.x = minX;
	}
	if (gSecondRobotPosition.x > maxX) {
		gSecondRobotPosition.x = maxX;
	}
	gRobotPosition.z = getRobotLaneCenterZWorld();
	gSecondRobotPosition.z = getLaneCenterZWorld(kSecondPlayerLaneIndex);
	robot.theta[robot.Torso] = gPlayerYaw;

	gRobotVelocityY -= kGravity * deltaTime;
	gRobotPosition.y += gRobotVelocityY * deltaTime;
	if (gRobotPosition.y < getRobotStandY()) {
		gRobotPosition.y = getRobotStandY();
		gRobotVelocityY = 0.0f;
	}
	gSecondRobotPosition.y = getRobotStandY();

	updateRaceState(deltaTime);
}


void cleanData() {
	
	// 释放内存
	delete camera;
	camera = NULL;

	for (int i=0; i<meshList.size(); i++) {
		meshList[i]->cleanData();
		delete meshList[i];
	}
	meshList.clear();

}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

int main(int argc, char **argv)
{
	// 初始化GLFW库，必须是应用程序调用的第一个GLFW函数
	glfwInit();

	// 配置GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// 配置窗口属性
	GLFWwindow* window = glfwCreateWindow(600, 600, "2023152022_郭洋_补充实验2", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	gWindow = window;
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	int framebufferWidth = 0;
	int framebufferHeight = 0;
	glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
	if (framebufferHeight > 0) {
		WIDTH = framebufferWidth;
		HEIGHT = framebufferHeight;
		camera->aspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
	}

	// 调用任何OpenGL的函数之前初始化GLAD
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Init mesh, shaders, buffer
	init();

	// 输出帮助信息
	printHelp();
	// 启用深度测试
	glEnable(GL_DEPTH_TEST);
	float lastFrame = static_cast<float>(glfwGetTime());
	while (!glfwWindowShouldClose(window))
	{
		float currentFrame = static_cast<float>(glfwGetTime());
		float deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glfwPollEvents();
		processMovement(window, deltaTime);
		display();

		// 交换颜色缓冲 以及 检查有没有触发什么事件（比如键盘输入、鼠标移动等）
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
	}

	cleanData();


	return 0;
}

// 每当窗口改变大小，GLFW会调用这个函数并填充相应的参数供你处理。
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
	WIDTH = width;
	HEIGHT = height;
	if (height > 0 && camera) {
		camera->aspect = static_cast<float>(width) / static_cast<float>(height);
	}
}
