#version 330 core

in vec3 position;
in vec3 normal;
in vec3 color;
in vec2 texCoord;

uniform int isShadow;
uniform int useTexture;
uniform int useLighting;
uniform sampler2D tex;
uniform vec2 texScale;
uniform vec2 texOffset;
uniform vec3 colorTint;
uniform float alpha;
uniform float shadowAlpha;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform float ambientStrength;
uniform float specStrength;
uniform float shininess;
uniform vec3 eye_position;

out vec4 fColor;

void main()
{
	if (isShadow == 1) {
		fColor = vec4(0.0, 0.0, 0.0, shadowAlpha);
	}
	else {
		vec4 baseColor = vec4(color, 1.0);
		if (useTexture == 1) {
			baseColor = texture(tex, texCoord * texScale + texOffset);
		}
		baseColor.rgb *= colorTint;
		baseColor.a *= alpha;
		if (useLighting == 1) {
			vec3 norm = normalize(normal);
			vec3 lightDir = normalize(lightPos - position);
			float diff = max(dot(norm, lightDir), 0.0);
			vec3 viewDir = normalize(eye_position - position);
			vec3 reflectDir = reflect(-lightDir, norm);
			float spec = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
			vec3 ambient = ambientStrength * lightColor;
			vec3 diffuse = diff * lightColor;
			vec3 specular = specStrength * spec * lightColor;
			vec3 lighting = ambient + diffuse + specular;
			fColor = vec4(baseColor.rgb * lighting, baseColor.a);
		}
		else {
			fColor = baseColor;
		}
	}
}
