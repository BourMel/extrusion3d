#include "meshquad.h"
#include "matrices.h"


void MeshQuad::clear()
{
    m_points.clear();
    m_quad_indices.clear();
}

int MeshQuad::add_vertex(const Vec3& P)
{
    m_points.push_back(P);
    return (m_points.size() -1);
}


void MeshQuad::add_quad(int i1, int i2, int i3, int i4)
{
    m_quad_indices.push_back(i1);
    m_quad_indices.push_back(i2);
    m_quad_indices.push_back(i3);
    m_quad_indices.push_back(i4);
}

void MeshQuad::convert_quads_to_tris(const std::vector<int>& quads, std::vector<int>& tris)
{
	tris.clear();
	tris.reserve(3*quads.size()/2);

	// Pour chaque quad on genere 2 triangles
	// Attention a repecter l'orientation des triangles

    //tester multiple de 4 ?
    for(int i=0; i<quads.size(); i+=4) {
        tris.push_back(quads[i]);
        tris.push_back(quads[i+1]);
        tris.push_back(quads[i+2]);

        tris.push_back(quads[i]);
        tris.push_back(quads[i+2]);
        tris.push_back(quads[i+3]);
    }

}

void MeshQuad::convert_quads_to_edges(const std::vector<int>& quads, std::vector<int>& edges)
{
	edges.clear();
	edges.reserve(quads.size()); // ( *2 /2 !)
	// Pour chaque quad on genere 4 aretes, 1 arete = 2 indices.
	// Mais chaque arete est commune a 2 quads voisins !
	// Comment n'avoir qu'une seule fois chaque arete ?

    //ici : toutes les arêtes implémentées (donc présence de doubles)
    for(int i=0; i<quads.size(); i+=4) {
        edges.push_back(quads[i]);
        edges.push_back(quads[i+1]);

        edges.push_back(quads[i+1]);
        edges.push_back(quads[i+2]);

        edges.push_back(quads[i+2]);
        edges.push_back(quads[i+3]);

        edges.push_back(quads[i+3]);
        edges.push_back(quads[i]);
    }
}

//PAS TESTE
void MeshQuad::bounding_sphere(Vec3& C, float& R)
{
    //pour la lisibilité
    int x = 0;
    int y = 1;
    int z = 2;

    int max_x = 0;
    int max_y = 0;
    int max_z = 0;
    int max_r = 0;

    for(int i=0; i<m_points.size(); i++) {
        if(m_points[i][x] > max_x) {
            max_x = m_points[i][x];
        }

        if(m_points[i][y] > max_y) {
            max_y = m_points[i][y];
        }

        if(m_points[i][z] > max_z) {
            max_z = m_points[i][z];
        }
    }

    max_r = std::max(max_x, max_y);
    max_r = std::max(max_z, max_r);

    C = Vec3(max_x/2, max_y/2, max_z/2);
    R = max_r;

    std::cout << "C: " << C << std::endl;
    std::cout << "R: " << R << std::endl;
}

//SENS TRIGO ?
void MeshQuad::create_cube()
{
	clear();

	// ajouter 8 sommets (-1 +1)

    int axeY = add_vertex(Vec3(0, 5, 0));
    int axeZ = add_vertex(Vec3(0, 0, 5));
    int axeX = add_vertex(Vec3(5, 0, 0));

    int p0 = add_vertex(Vec3(0, 0, 0));
    int p1 = add_vertex(Vec3(1, 0, 0));
    int p2 = add_vertex(Vec3(1, 0, 1));
    int p3 = add_vertex(Vec3(0, 0, 1));

    int ph0 = add_vertex(Vec3(0, 1, 0));
    int ph1 = add_vertex(Vec3(1, 1, 0));
    int ph2 = add_vertex(Vec3(1, 1, 1));
    int ph3 = add_vertex(Vec3(0, 1, 1));

    // ajouter 6 faces (sens trigo pour chacune) ??

    //dessus et dessous
    add_quad(p0, p1, p2, p3);
    add_quad(ph3, ph2, ph1, ph0);

    //avant et arrière
    add_quad(p0, ph0, ph1, p1);
    add_quad(p2, ph2, ph3, p3);

    //gauche et droite
    add_quad(p0, p3, ph3, ph0);
    add_quad(ph1, ph2, p2, p1);

    //repère
    add_quad(p0, axeY, axeY, p0);
    add_quad(p0, axeZ, axeZ, p0);
    add_quad(p0, axeX, axeX, p0);


	gl_update();
}

//PAS TESTE, pas utilisé
Vec3 MeshQuad::normal_of(const Vec3& A, const Vec3& B, const Vec3& C)
{
	// Attention a l'ordre des points !
	// le produit vectoriel n'est pas commutatif U ^ V = - V ^ U
	// ne pas oublier de normaliser le resultat.

    return Vec3(glm::normalize(glm::cross(A,B)));
}

//PAS TESTE, pas utilisé
bool MeshQuad::is_points_in_quad(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// On sait que P est dans le plan du quad.

	// P est-il au dessus des 4 plans contenant chacun la normale au quad et une arete AB/BC/CD/DA ?
	// si oui il est dans le quad

    int x = 0;
    int y = 1;
    int z = 2;

    Vec3 AB = B-A;
    Vec3 BC = C-B;
    Vec3 CD = D-C;
    Vec3 DA = A-D;

    //plan n°1 : normale puis déterminant
    Vec3 normale1 = glm::cross(AB,P);
    int res1 = normale1[x]*P[x] + normale1[y]*P[y] + normale1[z]*P[z];

    if(res1 < 0) return false;

    //plan n°2
    Vec3 normale2 = glm::cross(BC, P);
    int res2 = normale2[x]*P[x] + normale2[y]*P[y] + normale2[z]*P[z];

    if(res2 < 0) return false;

    //plan n°3
    Vec3 normale3 = glm::cross(CD, P);
    int res3 = normale3[x]*P[x] + normale3[y]*P[y] + normale3[z]*P[z];

    if(res3 < 0) return false;

    //plan n°4
    Vec3 normale4 = glm::cross(DA, P);
    int res4 = normale4[x]*P[x] + normale4[y]*P[y] + normale4[z]*P[z];

    if(res4 < 0) return false;

    return true;
}

//PAS TESTE
bool MeshQuad::intersect_ray_quad(const Vec3& P, const Vec3& Dir, int q, Vec3& inter)
{
    int x = 0;
    int y = 1;
    int z = 2;

    // recuperation des indices de points
    int i0 = m_quad_indices[q];
    int i1 = m_quad_indices[q+1];
    int i2 = m_quad_indices[q+2];
    int i3 = m_quad_indices[q+3];
    // recuperation des points
    Vec3 p0 = m_points[i0];
    Vec3 p1 = m_points[i1];
    Vec3 p2 = m_points[i2];
    Vec3 p3 = m_points[i3];

	// calcul de l'equation du plan (N+d)

    //deux vecteurs directeurs du plan
    Vec3 v = p1 - p0;
    Vec3 u = p3 - p2;
    //normale = leur produit vectoriel
    Vec3 N = glm::cross(u, v);
    //on calcule d en remplaçant x, y et z par les coordonnées de p0, qui appartient au plan
    int d = N[x]*p0[x] + N[y]*p0[y] + N[z]*p0[z];

    // calcul de l'intersection rayon plan

    // I = P + alpha*Dir est dans le plan => calcul de alpha
    // on remplace x, y, z dans l'équation du plan par les coordonnées paramétriques du rayon, ce qui nous donne t
    int t = -(N[x]*P[x] + N[y]+P[y] + N[z]+P[z] + d) / (N[x]*Dir[x] + N[y]*Dir[y] + N[z]*Dir[z]);
    // alpha => calcul de I
    //on remplace t dans l'équation du rayon pour trouver l'intersection I
    Vec3 I = Vec3(P[x]+Dir[x]*t, P[y]+Dir[y]*t, P[z]+Dir[z]*t);

	// I dans le quad ?

    if(is_points_in_quad(I, p0, p1, p2, p3)) {
        inter = I;
        return true;
    }

    return false;
}


int MeshQuad::intersected_closest(const Vec3& P, const Vec3& Dir)
{
	// on parcours tous les quads
	// on teste si il y a intersection avec le rayon
	// on garde le plus proche (de P)

    int inter = -1;

	return inter;
}


Mat4 MeshQuad::local_frame(int q)
{
	// Repere locale = Matrice de transfo avec
	// les trois premieres colones: X,Y,Z locaux
	// la derniere colonne l'origine du repere

	// ici Z = N et X = AB
	// Origine le centre de la face
	// longueur des axes : [AB]/2

	// recuperation des indices de points
	// recuperation des points

	// calcul de Z:N / X:AB -> Y

	// calcul du centre

	// calcul de la taille

	// calcul de la matrice

    return Mat4();
}

void MeshQuad::extrude_quad(int q)
{
	// recuperation des indices de points

	// recuperation des points

	// calcul de la normale

	// calcul de la hauteur

	// calcul et ajout des 4 nouveaux points

	// on remplace le quad initial par le quad du dessu

	// on ajoute les 4 quads des cotes

   gl_update();
}

void MeshQuad::transfo_quad(int q, const glm::mat4& tr)
{
	// recuperation des indices de points
	// recuperation des (references de) points

	// generation de la matrice de transfo globale:
	// indice utilisation de glm::inverse() et de local_frame

	// Application au 4 points du quad
}

void MeshQuad::decale_quad(int q, float d)
{

}

void MeshQuad::shrink_quad(int q, float s)
{

}

void MeshQuad::tourne_quad(int q, float a)
{

}





MeshQuad::MeshQuad():
	m_nb_ind_edges(0)
{}


void MeshQuad::gl_init()
{
	m_shader_flat = new ShaderProgramFlat();
	m_shader_color = new ShaderProgramColor();

	//VBO
	glGenBuffers(1, &m_vbo);

	//VAO
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_flat->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_flat->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &m_vao2);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_color->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_color->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	//EBO indices
	glGenBuffers(1, &m_ebo);
	glGenBuffers(1, &m_ebo2);
}

void MeshQuad::gl_update()
{
	//VBO
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_points.size() * sizeof(GLfloat), &(m_points[0][0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	std::vector<int> tri_indices;
	convert_quads_to_tris(m_quad_indices,tri_indices);

	//EBO indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,tri_indices.size() * sizeof(int), &(tri_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	std::vector<int> edge_indices;
	convert_quads_to_edges(m_quad_indices,edge_indices);
	m_nb_ind_edges = edge_indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,m_nb_ind_edges * sizeof(int), &(edge_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void MeshQuad::set_matrices(const Mat4& view, const Mat4& projection)
{
	viewMatrix = view;
	projectionMatrix = projection;
}

void MeshQuad::draw(const Vec3& color)
{
	glEnable(GL_CULL_FACE);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	m_shader_flat->startUseProgram();
	m_shader_flat->sendViewMatrix(viewMatrix);
	m_shader_flat->sendProjectionMatrix(projectionMatrix);
	glUniform3fv(m_shader_flat->idOfColorUniform, 1, glm::value_ptr(color));
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo);
	glDrawElements(GL_TRIANGLES, 3*m_quad_indices.size()/2,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_flat->stopUseProgram();

	glDisable(GL_POLYGON_OFFSET_FILL);

	m_shader_color->startUseProgram();
	m_shader_color->sendViewMatrix(viewMatrix);
	m_shader_color->sendProjectionMatrix(projectionMatrix);
	glUniform3f(m_shader_color->idOfColorUniform, 0.0f,0.0f,0.0f);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo2);
	glDrawElements(GL_LINES, m_nb_ind_edges,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_color->stopUseProgram();
	glDisable(GL_CULL_FACE);
}

