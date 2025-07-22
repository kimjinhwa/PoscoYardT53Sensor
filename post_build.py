import shutil
import os
import subprocess
import glob
Import("env")

def after_build(source, target, env):
    # 여기서 컴파일된 파일 처리
    print("Build completed!")
    get_git_info()
    copy_firmware_to_backup()
    print("복사 완료")
    # firmware.bin 등이 생성된 후 실행됨

# 실제 빌드 완료 후 실행되도록 콜백 등록
env.AddPostAction("buildprog", after_build)

def get_git_info():
    try:
        firmware_path = env.get("PROJECT_BUILD_DIR")
        build_path = env.get("PIOENV")
        source_directory = firmware_path 
        source_directory = os.path.join(source_directory, build_path)
        print(env.get("PROJECT_BUILD_DIR"))
        print(env.get("BUILD_DIR"))
        branch_name = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).strip().decode()
        commit_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip().decode()
        pioenv= env.get("PIOENV")
        print("build_dir : " + build_path)
        print("branch_name : " + branch_name) 
        print("commit_hash : " + commit_hash) 
        print("pioenv : " + pioenv) 
        changefirmware = os.path.join(source_directory, "firmware.bin")
        print("changefirmware : " + changefirmware)
        if os.path.exists(changefirmware):
            print("이름변경 시작")
            # new_name = f"firmware_{pioenv}_{branch_name}{commit_hash}.bin"
            new_name="firmware_T53.bin"

            new_path = os.path.join(source_directory, new_name)
            
            # 파일 접근 문제 해결을 위한 수정
            try:
                if os.path.exists(new_path):
                    os.remove(new_path)
                shutil.copy2(changefirmware, new_path)
                os.remove(changefirmware)
                print(f"Firmware renamed to: {new_name}")
            except Exception as copy_error:
                print(f"Error during copy/delete: {str(copy_error)}")
            
            # 백업 복사 등 추가 작업
            copy_firmware_to_backup()
        # env.Replace(PROGNAME="firmware_%s_%s%s" %  (pioenv,branch_name , commit_hash) )
 
        return branch_name, commit_hash,pioenv
    except Exception as e:
        print("Error retrieving Git information:", str(e))
        return None, None, None

def copy_firmware_to_backup(*args, **kwargs):

    firmware_path = env.get("PROJECT_BUILD_DIR")
    build_path = env.get("PIOENV")
    source_directory = firmware_path 
    source_directory = os.path.join(source_directory,build_path );
    target_directory = os.path.abspath(os.path.join(source_directory, os.pardir))
    target_directory = os.path.abspath(os.path.join(target_directory , os.pardir))
    target_directory = os.path.abspath(os.path.join(target_directory , os.pardir))
    target_directory = os.path.join(target_directory ,'uploadFirmware')
    bin_files = glob.glob(os.path.join(source_directory,'*.bin'))
    if not os.path.exists(target_directory):
        os.makedirs(target_directory)
    
    print("source_directory" + source_directory)
    print("target_directory " + target_directory)
    for bin_file in bin_files :
        print(target_directory)
        filename = os.path.basename(bin_file)
        targat_path = os.path.join(target_directory,filename)
        shutil.copy(bin_file, target_directory)
# Call the function to copy the firmware to the backup directory
