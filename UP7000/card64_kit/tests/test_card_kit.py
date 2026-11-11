import io
import json
import os
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import CardError, validate_volume, uuid_checked, verify_mount
from activate import new_fstab, patch_uuid_source, backup_files, restore_files
from probe import test_filesystem
from seed import safe_unpack

UUID = '2c7e86cf-85b3-4d45-a827-bfb75175221a'


def card():
    return {'name':'/dev/sdc1','type':'part','size':63_000_000_000,'disk_size':64_000_000_000,
            'disk_name':'/dev/sdc','fstype':'ext4','uuid':UUID,'ro':False,
            'transport':'usb','mountpoints':[None]}


class DeviceChecks(unittest.TestCase):
    def test_accept_64gb_data_partition(self):
        row=card();self.assertEqual(validate_volume(row,[row]),row)

    def test_reject_operating_system_on_same_disk(self):
        row=card();system={**row,'name':'/dev/mapper/root','type':'lvm','mountpoints':['/']}
        with self.assertRaises(CardError):validate_volume(row,[row,system])

    def test_reject_hdd_ssd_lvm_and_non_native_filesystem(self):
        for changes in [{'size':320_000_000_000},{'disk_size':4_000_000_000_000},
                        {'fstype':'exfat'},{'ro':True},{'type':'lvm'}, {'transport':'sata'}]:
            row={**card(),**changes}
            with self.subTest(changes=changes),self.assertRaises(CardError):validate_volume(row,[row])

    def test_uuid_is_not_shell_or_path_input(self):
        for value in ['../../sda','UUID=foo','$(reboot)','',None]:
            with self.subTest(value=value),self.assertRaises(CardError):uuid_checked(value)

    def test_missing_wrong_or_readonly_mount_cannot_fall_back(self):
        for info in [None, {'uuid':'wrong','fstype':'ext4','options':'rw'},
                     {'uuid':UUID,'fstype':'ext4','options':'ro'}]:
            with self.subTest(info=info),patch('common.mount_info',return_value=info),self.assertRaises(CardError):
                verify_mount(Path('/unused'),UUID)


class PersistentConfiguration(unittest.TestCase):
    def test_fstab_preserves_unrelated_system_mounts_and_comments(self):
        original='# comment\nUUID=root / ext4 defaults 0 1\nUUID=old /mnt/droneup ext4 defaults 0 2\n'
        new=new_fstab(original,UUID)
        self.assertIn('UUID=root / ext4 defaults 0 1',new)
        active=[line for line in new.splitlines() if not line.startswith('#') and '/mnt/droneup' in line]
        self.assertEqual(len(active),1);self.assertIn('UUID='+UUID,active[0])

    def test_refuse_ambiguous_mount_table(self):
        with self.assertRaises(CardError):new_fstab('UUID=a / ext4 defaults 0 1\n',UUID)

    def test_uuid_patching_preserves_code_and_is_idempotent(self):
        source="import json\nfrom pathlib import Path\nUUID = '896b17fa-8fe0-4111-853b-57b93ec87c60'\nprint('same behavior')\n"
        patched=patch_uuid_source(source)
        compile(patched,'patched','exec')
        self.assertIn("print('same behavior')",patched)
        self.assertEqual(patch_uuid_source(patched),patched)
        with self.assertRaises(CardError):patch_uuid_source('UUID = other_backend()')

    def test_rollback_restores_original_file_and_removes_only_new_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);original=root/'source.py';original.write_text('original')
            config=root/'new_config.json';backup=root/'backup'
            with patch('activate.PATHS',[str(original),str(config)]),patch('activate.CONFIG',config),\
                 patch('activate.command',return_value=''),\
                 patch('activate.subprocess.run',return_value=SimpleNamespace(stdout='enabled\n')):
                backup_files(backup)
                original.write_text('new code');config.write_text('new card')
                restore_files(backup)
            self.assertEqual(original.read_text(),'original');self.assertFalse(config.exists())


class DataIntegrity(unittest.TestCase):
    def test_probe_preserves_existing_files_and_removes_only_its_successful_test(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'user.txt').write_text('keep exactly')
            result=test_filesystem(root,6*1024**2,file_size=2*1024**2)
            self.assertEqual(result['written_bytes'],result['verified_bytes'])
            self.assertTrue(result['sha256_match'])
            self.assertEqual(list(root.iterdir()),[root/'user.txt'])
            self.assertEqual((root/'user.txt').read_text(),'keep exactly')

    def test_checksum_failure_keeps_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with patch('probe.hash_uncached',return_value='wrong'),self.assertRaises(CardError):
                test_filesystem(root,1024**2,file_size=1024**2)
            self.assertTrue(list(root.glob('.up7000_probe_*/part_000000.bin')))

    def test_archive_rejects_escape_and_symlink(self):
        for name,kind in [('../escape',tarfile.REGTYPE),('onboard_20261107/link',tarfile.SYMTYPE)]:
            with tempfile.TemporaryDirectory() as tmp:
                archive=Path(tmp)/'input.tar.gz';out=Path(tmp)/'out';out.mkdir()
                with tarfile.open(archive,'w:gz') as f:
                    info=tarfile.TarInfo(name);info.type=kind
                    if kind==tarfile.SYMTYPE:info.linkname='/etc'
                    f.addfile(info,io.BytesIO())
                with self.assertRaises(CardError):safe_unpack(archive,out)
                self.assertFalse((Path(tmp)/'escape').exists())


if __name__=='__main__':unittest.main()
