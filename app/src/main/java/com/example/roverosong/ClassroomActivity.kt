package com.example.roverosong

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.ImageView
import androidx.appcompat.app.AppCompatActivity

class ClassroomActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_classroom)

        // 전체화면 모드 설정
        window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                )

        // 돋보기 아이콘 클릭 이벤트 설정
        findViewById<ImageView>(R.id.searchIcon).setOnClickListener {
            // 다음 화면(NextActivity)으로 이동
            val intent = Intent(this, GuideActivity::class.java)
            startActivity(intent)
        }
    }
}
