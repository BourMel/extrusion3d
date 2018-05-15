
#include <QApplication>
#include <QGLViewer/simple_viewer.h>
#include <matrices.h>
#include <primitives.h>
#include <meshquad.h>

const Vec3 ROUGE   = {1,0,0};
const Vec3 VERT    = {0,1,0};
const Vec3 BLEU    = {0,0,1};
const Vec3 JAUNE   = {1,1,0};
const Vec3 CYAN    = {0,1,1};
const Vec3 MAGENTA = {1,0,1};
const Vec3 BLANC   = {1,1,1};
const Vec3 GRIS    = {0.5,0.5,0.5};
const Vec3 NOIR    = {0,0,0};



void draw_repere(const Primitives& prim, const Mat4& tr)
{
    Mat4 scaledTr = tr*scale(0.5)*translate(0, 0, 0.5);

    prim.draw_sphere(scaledTr, BLANC);

    prim.draw_cylinder(
                scaledTr*translate(0, 0, 1)*scale(0.5, 0.5, 1),
                BLEU);
    prim.draw_cone(
                scaledTr*translate(0, 0, 2),
                BLEU);

    prim.draw_cylinder(
                scaledTr*rotateX(90)*translate(0, 0, 1)*scale(0.5, 0.5, 1),
                VERT);
    prim.draw_cone(
                scaledTr*rotateX(90)*translate(0, 0, 2),
                VERT);

    prim.draw_cylinder(
                scaledTr*rotateY(90)*translate(0, 0, 1)*scale(0.5, 0.5, 1),
                ROUGE);
    prim.draw_cone(
                scaledTr*rotateY(90)*translate(0, 0, 2),
                ROUGE);
}


void star(MeshQuad& m)
{
    unsigned int length = 10;
    float rotate = 7;
    float shrink = 0.9;
    float maximise = 3;
    float decale = -0.8;

    m.create_cube();

    //on agrandit le cube au préalable pour éviter des valeurs trop petites
    for(unsigned int i=0; i<24; i+=4) {
         m.shrink_quad(i, maximise);
    }

    //pour chaque face du cube initial
    for(unsigned int i=0; i<24; i+=4) {
        //en respectant la longueur indiquée
        for(unsigned int l=0; l<length; l++) {
            m.extrude_quad(i);
            m.tourne_quad(i, rotate);
            m.shrink_quad(i, shrink);
            m.decale_quad(i, decale);
        }
    }
}






int main(int argc, char *argv[])
{
	Primitives prim;
    int selected_quad = -1;
	glm::mat4 selected_frame;
	MeshQuad mesh;

	// init du viewer
	QApplication a(argc, argv);
	SimpleViewer::init_gl();
	SimpleViewer viewer({0.1,0.1,0.1},5);

	// GL init
	viewer.f_init = [&] ()
	{
		prim.gl_init();
		mesh.gl_init();
	};

	// drawing
	viewer.f_draw = [&] ()
	{
		mesh.set_matrices(viewer.getCurrentModelViewMatrix(),viewer.getCurrentProjectionMatrix());
		prim.set_matrices(viewer.getCurrentModelViewMatrix(),viewer.getCurrentProjectionMatrix());

		mesh.draw(CYAN);

		if (selected_quad>=0)
			draw_repere(prim,selected_frame);
	};

	// to do when key pressed
	viewer.f_keyPress = [&] (int key, Qt::KeyboardModifiers mod)
	{
		switch(key)
		{
			case Qt::Key_C:
                if (!(mod & Qt::ControlModifier))
					mesh.create_cube();
				break;

            case Qt::Key_E:
                if(selected_quad != -1) {
                    mesh.extrude_quad(selected_quad);
                }
                break;

            case Qt::Key_Z:
                if(mod && Qt::Key_Shift) {
                    mesh.shrink_quad(selected_quad, 1.1);
                } else {
                    mesh.shrink_quad(selected_quad, 0.9);
                }
                break;

            case Qt::Key_T:
                if(mod && Qt::Key_Shift) {
                    mesh.tourne_quad(selected_quad, 1);
                } else {
                    mesh.tourne_quad(selected_quad, -1);
                }
                break;

            case Qt::Key_Plus:
                mesh.decale_quad(selected_quad, 0.5);
                break;
            case Qt::Key_Minus:
                mesh.decale_quad(selected_quad, -0.5);
                break;

			// generation d'objet
			case Qt::Key_S:
				star(mesh);
				break;
			// ....


			default:
				break;
		}

		Vec3 sc;
		float r;
		mesh.bounding_sphere(sc,r);
		viewer.setSceneCenter(qglviewer::Vec(sc[0],sc[1],sc[2]));
		viewer.setSceneRadius(r);
		viewer.camera()->centerScene();
		viewer.update();
	};

	// to do when mouse clicked (P + Dir = demi-droite (en espace objet) orthogonale à l'écran passant par le point cliqué
	viewer.f_mousePress3D = [&] (Qt::MouseButton /*b*/, const glm::vec3& P, const glm::vec3& Dir)
	{
		selected_quad = mesh.intersected_closest(P,Dir);
		if (selected_quad>=0)
			selected_frame = mesh.local_frame(selected_quad);
		std::cout << selected_quad << std::endl;
	};

	viewer.clearShortcuts();
	viewer.setShortcut(QGLViewer::EXIT_VIEWER,Qt::Key_Escape);
	viewer.show();
	return a.exec();
}
