#version 330 core

in vec3 vPosition;
in vec3 vColor;
in vec3 vNormal;
in vec2 vTexCoord;

out vec3 position;
out vec3 normal;
out vec3 color;
out vec2 texCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec4 v1 = model * vec4(vPosition, 1.0);
	vec4 v2 = vec4(v1.xyz / v1.w, 1.0);
	vec4 v3 = projection * view * v2;

	gl_Position = v3;

	position = vec3(v2.xyz);
	normal = vec3(model * vec4(vNormal, 0.0));
	color = vColor;
	texCoord = vTexCoord;
}
