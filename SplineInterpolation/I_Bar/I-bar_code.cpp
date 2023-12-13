#include <Eigen/Dense>
#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>

#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

using namespace std;

struct Triple
{
    float x;
    float y;
    float z;
};

struct Rotation {
    float x, y, z;
    float angle; //angle is in degrees
};

struct Quaternion {
    float s, x, y, z;
};

void init(void);
void init_lights(void);
void display(void);

void draw_frame();
void draw_IBar();
void parse_file();
void transform_interpolate();

void key_pressed(unsigned char key, int x, int y);

Rotation quat2rot(const Quaternion &q);
Quaternion rot2quat(const Rotation &r);
Quaternion normalized(const Quaternion &q);

float cam_position[] = {0, 0, 40};

float near_param = 1.f, far_param = 60.f,
    left_param = -1.f, right_param = 1.f,
    top_param = 1.f, bottom_param = -1.f;

const float light_color[3] = {0.7, 0.7, 0.7};
const float light_position[3] = {1, 1, 1};

const float ambient_reflect[3] = {1, 1, 1};
const float diffuse_reflect[3] = {1, 1, 1};
const float specular_reflect[3] = {1, 1, 1};
const float shininess = 5.f;

int num_frames;

int curr_frame;

vector<int> kframes;

vector<Triple> translations;

vector<Triple> scales;

vector<Quaternion> rotations;

Eigen::Matrix4f B;

int xres;
int yres;

float deg2rad(float angle)
{
    return angle * M_PI / 180.0;
}

float rad2deg(float angle)
{
    return angle / M_PI * 180.0;
}

Quaternion rot2quat(const Rotation &r) {
    float angle = deg2rad(r.angle);
    Quaternion q;
    q.s = cos(angle / 2);
    q.x = r.x * sin(angle / 2);
    q.y = r.y * sin(angle / 2);
    q.z = r.z * sin(angle / 2);
    return q;
}

Rotation quat2rot(const Quaternion &q) {
    float denom = sqrt(1 - pow(q.s, 2));
    if (denom == 0)
        return Rotation{1, 0, 0, 0};
    Rotation r;
    r.x = q.x / denom;
    r.y = q.y / denom;
    r.z = q.z / denom;
    r.angle = rad2deg(2 * acos(q.s));
    return r;
}

Quaternion normalized(const Quaternion &q) {
    float norm = sqrt(pow(q.s, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2));
    return Quaternion{q.s / norm, q.x / norm, q.y / norm, q.z / norm};
}

void init()
{
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_RESCALE_NORMAL);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);

    //Projection matrix
    glMatrixMode(GL_MODELVIEW);

    curr_frame = 0;

    B << 0, 2, 0, 0,
        -1, 0, 1, 0,
        2, -5, 4, -1,
        -1, 3, -3, 1;
    B /= 2;

    transform_interpolate();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

    draw_frame();

    glutSwapBuffers();
}

void drawPrism(GLdouble base, GLdouble height) {
    glBegin(GL_QUADS);
    glVertex3f(-base, base, height);
    glVertex3f(-base, -base, height);
    glVertex3f(base, -base, height);
    glVertex3f(base, base, height);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-base, -base, height);
    glVertex3f(-base, 0, 0);
    glVertex3f(base, 0, 0);
    glVertex3f(base, -base, height);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(base, -base, height);
    glVertex3f(base, -base, 0);
    glVertex3f(base, base, 0);
    glVertex3f(base, base, height);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-base, base, height);
    glVertex3f(-base, base, 0);
    glVertex3f(-base, -base, 0);
    glVertex3f(-base, -base, height);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(-base, base, 0);
    glVertex3f(-base, -base, 0);
    glVertex3f(base, -base, 0);
    glVertex3f(base, base, 0);
    glEnd();

    glBegin(GL_QUADS);
    glVertex3f(base, base, height);
    glVertex3f(base, base, 0);
    glVertex3f(-base, base, 0);
    glVertex3f(-base, base, height);
    glEnd();
}

void drawIBar()
{
    float cyRad = 0.2, cyHeight = 1.0;
    int quadStacks = 4, quadSlices = 4;
    
    glPushMatrix();
    glColor3f(0, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 1, 0, 0);
    drawPrism(cyRad, 2.0 * cyHeight);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    drawPrism(cyRad, cyHeight);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    drawPrism(cyRad, cyHeight);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    drawPrism(cyRad, cyHeight);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    drawPrism(cyRad, cyHeight);
    glPopMatrix();
}

void key_pressed(unsigned char key, int x, int y)
{
    if (key == 'q')
    {
        exit(0);
    }
    else if (key == 'f')
    {
        curr_frame = (curr_frame + 1) % num_frames;
        glutPostRedisplay();
    }
}

void draw_frame() {
    Triple translation = translations[curr_frame];
    Triple scale = scales[curr_frame];
    Rotation rotation = quat2rot(rotations[curr_frame]);

    glPushMatrix();
    glTranslatef(translation.x, translation.y, translation.z);
    glScalef(scale.x, scale.y, scale.z);
    glRotatef(rotation.angle, rotation.x, rotation.y, rotation.z);
    drawIBar();
    glPopMatrix();
}

void parse_file(const string &filename)
{
    ifstream infile(filename);
    string line;

    getline(infile, line);
    num_frames = stoi(line);

    translations = vector<Triple>(num_frames);
    scales = vector<Triple>(num_frames);
    rotations = vector<Quaternion>(num_frames);

    while (!line.empty()) {
        istringstream iss(line);
        string type;
        iss >> type;

        int curr_kframe;

        if (type == "Frame") {
            iss >> curr_kframe;
            kframes.push_back(curr_kframe);
        }
        else if (type == "rotation") {
            float rx, ry, rz, angle;
            iss >> rx >> ry >> rz >> angle;

            Rotation r{rx, ry, rz, angle};
            rotations[curr_kframe] = normalized(rot2quat(r));
        }
        else if (type == "scale") {
            float sx, sy, sz;
            iss >> sx >> sy >> sz;

            scales[curr_kframe] = Triple{sx, sy, sz};
        }
        else if (type == "translation") {
            float tx, ty, tz;
            iss >> tx >> ty >> tz;

            translations[curr_kframe] = Triple{tx, ty, tz};
        }
        getline(infile, line);
    }
}

float compute_f(const float u, const Eigen::Vector4f &vec_p) {
    Eigen::Vector4f vec_u{1, u, pow(u, 2), pow(u, 3)};
    return vec_u.dot(B * vec_p);
}

Triple triple_interpolate(const float u, const Triple &t1, const Triple &t2,
                            const Triple &t3, const Triple &t4) {
    Triple t;
    t.x = compute_f(u, Eigen::Vector4f{t1.x, t2.x, t3.x, t4.x});
    t.y = compute_f(u, Eigen::Vector4f{t1.y, t2.y, t3.y, t4.y});
    t.z = compute_f(u, Eigen::Vector4f{t1.z, t2.z, t3.z, t4.z});
    return t;
}

Quaternion quaternion_interpolate(const float u, const Quaternion &q1,
                                    const Quaternion &q2, const Quaternion &q3,
                                    const Quaternion &q4) {
    Quaternion q;
    q.s = compute_f(u, Eigen::Vector4f{q1.s, q2.s, q3.s, q4.s});
    q.x = compute_f(u, Eigen::Vector4f{q1.x, q2.x, q3.x, q4.x});
    q.y = compute_f(u, Eigen::Vector4f{q1.y, q2.y, q3.y, q4.y});
    q.z = compute_f(u, Eigen::Vector4f{q1.z, q2.z, q3.z, q4.z});
    return normalized(q);
}

void transform_interpolate() {
    int num_kframes = kframes.size();

    for (int i = 0; i < num_kframes; i++) {
        int kframe1 = kframes[i % num_kframes];
        int kframe2 = kframes[(i + 1) % num_kframes]; 
        int kframe0 = kframes[(num_kframes + i - 1) % num_kframes];
        int kframe3 = kframes[(i + 2) % num_kframes];
        int max_frame = (num_frames + kframe2 - 1) % num_frames;
        for (int f = kframe1 + 1; f <= max_frame; f++) {
            float u = (float)(f - kframe1) / (max_frame + 1 - kframe1);
            assert(u > 0 && u < 1);

            translations[f] = triple_interpolate(u, translations[kframe0], translations[kframe1],
                                                translations[kframe2], translations[kframe3]);

            scales[f] = triple_interpolate(u, scales[kframe0], scales[kframe1],
                                        scales[kframe2], scales[kframe3]);

            rotations[f] = quaternion_interpolate(u, rotations[kframe0], rotations[kframe1],
                                                  rotations[kframe2], rotations[kframe3]);
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc != 4) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }

    string kframe_filename = argv[1];
    xres = stoi(argv[2]);
    yres = stoi(argv[3]);

    parse_file(kframe_filename);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Animation");
    

    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(key_pressed);
    glutMainLoop();
}
