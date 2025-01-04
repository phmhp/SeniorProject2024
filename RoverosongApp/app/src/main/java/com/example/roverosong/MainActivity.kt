package com.example.roverosong

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.Button
import com.google.firebase.FirebaseApp
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {
    private lateinit var database: DatabaseReference // Firebase Database 참조

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Firebase Realtime Database의 UserResults 참조
        database = FirebaseDatabase.getInstance().getReference("UserResults")

        // 전체화면 모드 설정
        window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                )

        // 버튼 이벤트 처리
        setupButtonListeners()
    }

    private fun setupButtonListeners() {
        // 강의실 버튼 클릭 이벤트
        findViewById<Button>(R.id.classroomButton).setOnClickListener {
            sendDataToFirebase("r_el") // Firebase에 데이터 전송
            navigateToGuideActivity() // GuideActivity로 이동
        }

        // 화장실 버튼 클릭 이벤트
        findViewById<Button>(R.id.restroomButton).setOnClickListener {
            sendDataToFirebase("p_el")
            navigateToGuideActivity()
        }

        // 주차장 버튼 클릭 이벤트
        findViewById<Button>(R.id.parkingButton).setOnClickListener {
            sendDataToFirebase("parking")
            navigateToGuideActivity()
        }

        // 미술관 버튼 클릭 이벤트
        findViewById<Button>(R.id.artGalleryButton).setOnClickListener {
            sendDataToFirebase("art_gallery")
            navigateToGuideActivity()
        }
    }

    private fun sendDataToFirebase(selectedOption: String) {
        // 선택한 옵션을 Firebase에 저장
        database.child("requiredIngredients").setValue(selectedOption)
            .addOnSuccessListener {
                println("Data successfully sent to Firebase: $selectedOption")
            }
            .addOnFailureListener {
                println("Failed to send data to Firebase: ${it.message}")
            }
    }

    private fun navigateToGuideActivity() {
        // GuideActivity로 화면 전환
        val intent = Intent(this, GuideActivity::class.java)
        startActivity(intent)
    }
}
