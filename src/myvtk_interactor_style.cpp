#include "include/myvtk_interactor_style.h"

#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>                // ✅ 修复 CurrentRenderer 错误
#include <vtkActor.h>
#include <vtkActorCollection.h>         // ✅ 修复 GetActors() 和 vtkActorCollection
#include <vtkCollection.h>              // ✅ 修复 vtkCollectionSimpleIterator
#include <vtkProperty.h>

vtkStandardNewMacro(MyVtkInteractorStyle);

void MyVtkInteractorStyle::OnRightButtonDown() {
}

void MyVtkInteractorStyle::OnRightButtonUp() {
    
}

void MyVtkInteractorStyle::OnChar() {
    vtkRenderWindowInteractor* rwi = this->Interactor;
    char key = rwi->GetKeyCode();  // 获取按键字符

    switch (key) {
        case '=':
        case '+': {
            // 增大点的尺寸
            if (this->CurrentRenderer) {
                vtkActorCollection* ac = this->CurrentRenderer->GetActors();
                vtkCollectionSimpleIterator ait;
                for (ac->InitTraversal(ait); vtkActor* anActor = ac->GetNextActor(ait); ) {
                    double currentSize = anActor->GetProperty()->GetPointSize();
                    double newSize = currentSize + 1.0;
                    anActor->GetProperty()->SetPointSize(newSize);
                }
                rwi->Render();
            }
            break;
        }

        case '-': {
            // 减小点的尺寸
            if (this->CurrentRenderer) {
                vtkActorCollection* ac = this->CurrentRenderer->GetActors();
                vtkCollectionSimpleIterator ait;
                for (ac->InitTraversal(ait); vtkActor* anActor = ac->GetNextActor(ait); ) {
                    double currentSize = anActor->GetProperty()->GetPointSize();
                    double newSize = (currentSize > 1.0) ? currentSize - 1.0 : 1.0;
                    anActor->GetProperty()->SetPointSize(newSize);
                }
                rwi->Render();
            }
            break;
        }

        default:
            // ⭐️ 关键：把其他按键交给父类处理
            this->Superclass::OnChar();
            break;
    }
}
